// Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0

// NOLINTBEGIN

/**
 * @file act_evaluate.cpp
 * @brief ACT ONNX offline and SO-101 real-robot evaluation entry.
 *
 * This is the C++ twin of act_evaluate.py. It runs an exported ACT ONNX graph
 * and reproduces the runtime pieces outside the graph: input normalization,
 * action unnormalization, SO-101 calibration, and action queue playback.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "onnxruntime_cxx_api.h"
#include "spacemit_ort_env.h"
#include "utils/common.h"
#include "utils/act_stats.h"

#ifdef ACT_ROBOT_HW
#include <opencv2/opencv.hpp>
extern "C" {
#include "motor.h"
}
#endif

using namespace std;
using namespace std::chrono;

static constexpr double kTwoPi = 2.0 * M_PI;
static constexpr double kMaxStep = 4095.0;  // STS3215: 4096 ticks/rev -> max index 4095

// --------------------------------------------------------------------------- //
// Ctrl+C handling. The control loop polls this flag and exits cleanly so the
// motors get idled and freed and the cameras released. Without it, SIGINT's
// default action kills the process mid-loop, leaving the arm under torque and
// the serial port / cameras held open (next run fails to open the device).
// The handler is async-signal-safe: it only sets the flag.
// --------------------------------------------------------------------------- //
static volatile sig_atomic_t g_stop = 0;
static void OnSignal(int) { g_stop = 1; }
static void InstallSignalHandlers() {
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = OnSignal;
    // No SA_RESTART: a pending blocking read returns EINTR so the loop can exit.
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

static void IgnoreSignalHandlers() {
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

// --------------------------------------------------------------------------- //
// Stats + calibration (parsed from act_norm_stats.txt)
// --------------------------------------------------------------------------- //
using Stats = ActStats;

// --------------------------------------------------------------------------- //
// lerobot SO-101 calibration math (mirrors motors_bus.py normalize/unnormalize)
//   DEGREES:     deg  = (raw - mid) * 360 / 4095     ; mid = (min + max)/2
//                raw  = deg * 4095 / 360 + mid
//   RANGE_0_100: norm = (raw - min) / (max - min) * 100
//                raw  = norm/100 * (max - min) + min
//   drive_mode == 1 flips sign (none do here, but handled for completeness).
// --------------------------------------------------------------------------- //
static double RawToNorm(const MotorCalib& c, int raw) {
    double out;
    if (c.norm_mode == "RANGE_0_100") {
        double span = double(c.range_max - c.range_min);
        out = span != 0.0 ? (double(raw - c.range_min) / span) * 100.0 : 0.0;
        if (c.drive_mode == 1) out = 100.0 - out;
    } else {  // DEGREES
        double mid = (c.range_min + c.range_max) / 2.0;
        out = (double(raw) - mid) * 360.0 / kMaxStep;
        if (c.drive_mode == 1) out = -out;
    }
    return out;
}

static int NormToRaw(const MotorCalib& c, double val, bool clamp_to_range) {
    double raw;
    if (c.norm_mode == "RANGE_0_100") {
        if (c.drive_mode == 1) val = 100.0 - val;
        val = std::min(100.0, std::max(0.0, val));
        raw = (val / 100.0) * double(c.range_max - c.range_min) + c.range_min;
    } else {  // DEGREES
        if (c.drive_mode == 1) val = -val;
        double mid = (c.range_min + c.range_max) / 2.0;
        raw = val * kMaxStep / 360.0 + mid;
    }
    int iraw = int(llround(raw));
    if (clamp_to_range) iraw = std::min(c.range_max, std::max(c.range_min, iraw));
    iraw = std::min(4095, std::max(0, iraw));  // hard servo bounds
    return iraw;
}

// --------------------------------------------------------------------------- //
// Normalization (numpy-identical)
// --------------------------------------------------------------------------- //
// img_rgb_hwc: H*W*3 float in [0,1]; writes CHW normalized into out (offset by base).
static void NormalizeImageInto(const vector<float>& img_rgb_hwc, int H, int W,
    const vector<float>& mean, const vector<float>& std,
    vector<float>& out, size_t base) {
    const size_t HW = size_t(H) * W;
    for (int ch = 0; ch < 3; ++ch) {
        float m = mean[ch], sd = std[ch];
        for (size_t i = 0; i < HW; ++i) {
            float v = img_rgb_hwc[i * 3 + ch];  // HWC -> pick channel
            out[base + size_t(ch) * HW + i] = (v - m) / sd;
        }
    }
}

static vector<float> NormalizeState(const vector<float>& state,
    const vector<float>& mean, const vector<float>& std) {
    vector<float> out(state.size());
    for (size_t i = 0; i < state.size(); ++i) out[i] = (state[i] - mean[i]) / std[i];
    return out;
}

static void UnnormalizeAction(const float* a, const vector<float>& mean,
    const vector<float>& std, vector<float>& out) {
    for (size_t i = 0; i < mean.size(); ++i) out[i] = a[i] * std[i] + mean[i];
}

static bool LooksLikeRawImageInput(const vector<float>& images) {
    if (images.empty()) return false;
    for (float v : images) {
        if (v < 0.0f || v > 1.0f) return false;
    }
    return true;
}

static void NormalizeLoadedImagesIfRaw(
    vector<float>& images, const vector<int64_t>& shape, const Stats& st) {
    if (!LooksLikeRawImageInput(images)) return;
    if (shape.size() != 4) return;
    const int n_cam = static_cast<int>(shape[0]);
    const int channels = static_cast<int>(shape[1]);
    const int height = static_cast<int>(shape[2]);
    const int width = static_cast<int>(shape[3]);
    if (channels != 3 || n_cam != static_cast<int>(st.cam_names.size())) return;

    const size_t hw = static_cast<size_t>(height) * width;
    for (int ci = 0; ci < n_cam; ++ci) {
        const auto& name = st.cam_names[ci];
        const auto& mean = st.image_mean.at(name);
        const auto& std = st.image_std.at(name);
        for (int ch = 0; ch < 3; ++ch) {
            const size_t base = (static_cast<size_t>(ci) * 3 + ch) * hw;
            for (size_t i = 0; i < hw; ++i) {
                images[base + i] = (images[base + i] - mean[ch]) / std[ch];
            }
        }
    }
}

// --------------------------------------------------------------------------- //
// Config / CLI
// --------------------------------------------------------------------------- //
struct Config {
    string model_path;
    string stats_path;
    // runtime
    bool use_ep = false;
    int threads = 4;
    string affinity;
    // robot
    string port = "/dev/ttyACM0";
    uint32_t baud = 1000000;
    map<string, int> cam_index;  // name -> /dev/videoN index
    double fps = 30.0;
    double episode_time = 180.0;
    int n_action_steps = 0;        // 0 = stats default
    float motor_speed = 3.0f;      // rad/s sent to motor lib
    bool no_clamp = false;         // disable clamping raw step to [range_min,range_max]
    bool dry_run = false;          // compute actions but DON'T send to motors
    bool verbose = false;
    // offline
    string images_npy;             // (n_cam,3,H,W) already in [0,1] OR raw? -> see note
    string state_npy;              // (state_dim) in lerobot-norm space (degrees/0..100)
};

static void PrintUsage(const char* p) {
    cout <<
    "Usage: " << p << " <model.onnx> --stats <act_norm_stats.txt> [options]\n"
    "  --stats P            norm stats + calibration (from export_norm_stats.py) [required]\n"
    "Runtime:\n"
    "  -s, --spacemit       Enable SpaceMIT EP (default CPU)\n"
    "  -t, --threads N      Intra/EP thread count (default 4)\n"
    "  -a, --affinity \"8;9\" SpaceMIT EP core affinity\n"
    "Robot (hardware build):\n"
    "  --port P             SO-101 serial port (default /dev/ttyACM0)\n"
    "  --baud N             Baud rate (default 1000000)\n"
    "  --cam NAME=IDX       Camera mapping, repeatable (e.g. --cam top=13 --cam wrist=15)\n"
    "  --fps F              Control loop rate (default 30)\n"
    "  --episode-time S     Episode length seconds (default 180)\n"
    "  --n-action-steps N   Actions per predicted chunk (0=stats default)\n"
    "  --motor-speed R      Motor target speed rad/s (default 3.0)\n"
    "  --no-clamp           Don't clamp raw step to [range_min,range_max]\n"
    "  --dry-run            Compute actions but do NOT command the motors\n"
    "  --verbose            Print per-step state/action\n"
    "Offline (no hardware):\n"
    "  --images-npy P       (n_cam,3,H,W) float32 raw [0,1] or normalized images\n"
    "  --state-npy P        (state_dim) float32 state in lerobot-norm space\n";
}

static bool ParseArgs(int argc, char** argv, Config& c) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return false;
    }
    string first = argv[1];
    if (first == "-h" || first == "--help") {
        PrintUsage(argv[0]);
        exit(0);
    }
    c.model_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        string a = argv[i];
        auto next = [&](const char* name) -> string {
            if (i + 1 >= argc) {
                cerr << name << " needs a value\n";
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--stats") c.stats_path = next("--stats");
        else if (a == "-s" || a == "--spacemit") c.use_ep = true;
        else if (a == "-t" || a == "--threads") c.threads = stoi(next("--threads"));
        else if (a == "-a" || a == "--affinity") c.affinity = next("--affinity");
        else if (a == "--port") c.port = next("--port");
        else if (a == "--baud") c.baud = (uint32_t)stoul(next("--baud"));
        else if (a == "--cam") {
            string kv = next("--cam");
            auto pos = kv.find('=');
            if (pos == string::npos) {
                cerr << "--cam expects NAME=IDX\n";
                return false;
            }
            c.cam_index[kv.substr(0, pos)] = stoi(kv.substr(pos + 1));
        } else if (a == "--fps") c.fps = stod(next("--fps"));
        else if (a == "--episode-time") c.episode_time = stod(next("--episode-time"));
        else if (a == "--n-action-steps") c.n_action_steps = stoi(next("--n-action-steps"));
        else if (a == "--motor-speed") c.motor_speed = stof(next("--motor-speed"));
        else if (a == "--no-clamp") c.no_clamp = true;
        else if (a == "--dry-run") c.dry_run = true;
        else if (a == "--verbose") c.verbose = true;
        else if (a == "--images-npy") c.images_npy = next("--images-npy");
        else if (a == "--state-npy") c.state_npy = next("--state-npy");
        else if (a == "-h" || a == "--help") {
            PrintUsage(argv[0]);
            exit(0);
        } else {
            cerr << "unknown arg: " << a << "\n";
            PrintUsage(argv[0]);
            return false;
        }
    }
    if (c.stats_path.empty()) {
        cerr << "--stats is required\n";
        return false;
    }
    return true;
}

// --------------------------------------------------------------------------- //
// ONNX session (same setup as act_benchmark.cpp)
// --------------------------------------------------------------------------- //
struct OnnxRunner {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "act_evaluate"};
    Ort::SessionOptions opts;
    unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions alloc;
    vector<string> in_names, out_names;
    Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

    OnnxRunner(const Config& cfg) {
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        opts.SetIntraOpNumThreads(cfg.threads);
        if (cfg.use_ep) {
            unordered_map<string, string> popts;
            popts["SPACEMIT_EP_INTRA_THREAD_NUM"] = to_string(cfg.threads);
            if (!cfg.affinity.empty()) popts["SPACEMIT_EP_INTRA_THREAD_AFFINITY"] = cfg.affinity;
            Ort::SessionOptionsSpaceMITEnvInit(opts, popts);
            opts.EnableMemPattern();
            opts.EnableCpuMemArena();
        }
        session = make_unique<Ort::Session>(env, cfg.model_path.c_str(), opts);
        for (size_t i = 0; i < session->GetInputCount(); ++i)
            in_names.push_back(session->GetInputNameAllocated(i, alloc).get());
        for (size_t i = 0; i < session->GetOutputCount(); ++i)
            out_names.push_back(session->GetOutputNameAllocated(i, alloc).get());
    }

    // images: (n_cam*3*H*W) normalized; state: (state_dim) normalized.
    // Returns the raw (normalized-space) action chunk (chunk_size * action_dim).
    vector<float> RunChunk(vector<float>& images, const vector<int64_t>& img_shape,
                            vector<float>& state, const vector<int64_t>& st_shape,
                            int& chunk, int& adim) {
        vector<Ort::Value> ins;
        vector<const char*> in_ptr, out_ptr;
        for (auto& n : in_names) {
            if (n == "images")
                ins.push_back(Ort::Value::CreateTensor<float>(
                    mem, images.data(), images.size(), img_shape.data(), img_shape.size()));
            else if (n == "state")
                ins.push_back(Ort::Value::CreateTensor<float>(
                    mem, state.data(), state.size(), st_shape.data(), st_shape.size()));
            else
                throw runtime_error("unhandled input: " + n);
        }
        for (auto& s : in_names) in_ptr.push_back(s.c_str());
        for (auto& s : out_names) out_ptr.push_back(s.c_str());
        auto outs = session->Run(Ort::RunOptions{nullptr}, in_ptr.data(), ins.data(),
            ins.size(), out_ptr.data(), out_ptr.size());
        auto shp = outs[0].GetTensorTypeAndShapeInfo().GetShape();  // (1, chunk, adim)
        chunk = (int)shp[1];
        adim = (int)shp[2];
        const float* p = outs[0].GetTensorData<float>();
        return vector<float>(p, p + size_t(chunk) * adim);
    }
};

struct LocalTimer {
    high_resolution_clock::time_point t0;
    void Start() { t0 = high_resolution_clock::now(); }
    double ElapsedMs() const {
        return duration_cast<microseconds>(high_resolution_clock::now() - t0).count() / 1000.0;
    }
};

// --------------------------------------------------------------------------- //
// Offline mode: inputs from npy, no hardware.
// --------------------------------------------------------------------------- //
static int RunOffline(const Config& cfg, const Stats& st, OnnxRunner& ort) {
    if (cfg.images_npy.empty() || cfg.state_npy.empty()) {
        cerr << "[offline] need --images-npy and --state-npy (or build with -DACT_ROBOT_HW=ON "
                "for real-robot mode)\n";
        return 2;
    }
    vector<int64_t> img_shape, st_shape;
    vector<float> images = LoadNpyF32(cfg.images_npy, img_shape);  // raw [0,1] or normalized
    vector<float> state = LoadNpyF32(cfg.state_npy, st_shape);  // (state_dim), lerobot-norm space
    NormalizeLoadedImagesIfRaw(images, img_shape, st);

    // The npy `state` is in lerobot-norm space (degrees/0..100); MEAN_STD normalize it.
    vector<float> state_n = NormalizeState(state, st.state_mean, st.state_std);

    // Reshape image npy to (1,n_cam,3,H,W) for the graph.
    vector<int64_t> img4 = {1};
    for (auto d : img_shape) img4.push_back(d);
    vector<int64_t> st2 = {1, (int64_t)state_n.size()};

    int chunk = 0, adim = 0;
    LocalTimer t;
    t.Start();
    auto raw = ort.RunChunk(images, img4, state_n, st2, chunk, adim);
    double infer_ms = t.ElapsedMs();

    vector<float> a0(adim);
    UnnormalizeAction(raw.data(), st.action_mean, st.action_std, a0);
    cout << "[offline] inference=" << infer_ms << "ms chunk=" << chunk << " adim=" << adim << "\n";
    cout << "[offline] action[0] (lerobot space): ";
    for (int i = 0; i < adim; ++i) cout << a0[i] << (i + 1 < adim ? ", " : "\n");

    // Show calibration round-trip -> raw step (what would go to the motors).
    if ((int)st.calib.size() == adim) {
        cout << "[offline] action[0] -> raw step: ";
        for (int i = 0; i < adim; ++i)
            cout << NormToRaw(st.calib[i], a0[i], !cfg.no_clamp)
                << (i + 1 < adim ? ", " : "\n");
    }
    return 0;
}

// --------------------------------------------------------------------------- //
// Hardware mode: OpenCV cameras + motor library.
// --------------------------------------------------------------------------- //
#ifdef ACT_ROBOT_HW
static int RunRobot(const Config& cfg, const Stats& st, OnnxRunner& ort) {
    const int n_cam = (int)st.cam_names.size();
    const int H = st.img_h, W = st.img_w, A = st.action_dim, S = st.state_dim;
    if ((int)st.calib.size() != S) {
        cerr << "calib count != state_dim\n";
        return 2;
    }

    // ---- Open cameras in model order ----
    vector<cv::VideoCapture> caps(n_cam);
    for (int i = 0; i < n_cam; ++i) {
        const string& name = st.cam_names[i];
        auto it = cfg.cam_index.find(name);
        if (it == cfg.cam_index.end()) {
            cerr << "missing --cam " << name << "=IDX\n";
            return 2;
        }
        cv::VideoCapture& cap = caps[i];
        cap.open(it->second, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            cerr << "cannot open camera " << name << " idx " << it->second << "\n";
            return 2;
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);
        cap.set(cv::CAP_PROP_FPS, cfg.fps);
        cout << "[robot] camera " << name << " <- /dev/video" << it->second << "\n";
    }

    // ---- Open 6 motors (ID from calibration) on one bus ----
    vector<struct motor_dev*> motors(S, nullptr);
    auto release_cameras = [&]() {
        for (auto& cap : caps) {
            if (cap.isOpened()) cap.release();
        }
    };
    int allocated_motors = 0;
    for (int i = 0; i < S; ++i) {
        motors[i] = motor_alloc_uart("drv_uart_feetech", cfg.port.c_str(), cfg.baud,
            (uint8_t)st.calib[i].id, nullptr);
        if (!motors[i]) {
            cerr << "motor_alloc failed: " << st.motor_order[i] << "\n";
            motor_free(motors.data(), (uint32_t)allocated_motors);
            release_cameras();
            return 2;
        }
        ++allocated_motors;
    }
    if (motor_init(motors.data(), S) != 0) {
        cerr << "motor_init failed\n";
        // motor_free releases handles returned by motor_alloc_uart; the
        // drv_uart_feetech free path does not require successful init.
        motor_free(motors.data(), (uint32_t)allocated_motors);
        release_cameras();
        return 2;
    }
    cout << "[robot] " << S << " motors on " << cfg.port << " @ " << cfg.baud << "\n";

    const int n_steps = std::min(cfg.n_action_steps > 0 ? cfg.n_action_steps : st.n_action_steps,
        st.chunk_size);

    // Buffers reused each loop.
    vector<float> images(size_t(n_cam) * 3 * H * W);
    vector<float> img_rgb(size_t(H) * W * 3);
    vector<int64_t> img_shape = {1, n_cam, 3, H, W};
    vector<int64_t> st_shape = {1, S};

    // Action queue: store rows of length A.
    vector<vector<float>> queue;
    size_t q_head = 0;

    vector<double> infer_ms, loop_ms;
    auto t_start = steady_clock::now();
    int step = 0;
    cout << "[robot] start: fps=" << cfg.fps << " n_action_steps=" << n_steps
        << " chunk=" << st.chunk_size << (cfg.dry_run ? " (DRY-RUN)\n" : "\n");

    while (!g_stop &&
        duration_cast<duration<double>>(steady_clock::now() - t_start).count() <
        cfg.episode_time) {
        auto loop_t0 = steady_clock::now();

        // ---- Read motors -> raw step -> lerobot-norm -> MEAN_STD ----
        vector<struct motor_state> ms(S);
        motor_get_states(motors.data(), ms.data(), S);
        vector<float> state(S);
        for (int i = 0; i < S; ++i) {
            int raw = (int)llround(double(ms[i].pos) / kTwoPi * kMaxStep);  // rad -> raw step
            raw = std::min(4095, std::max(0, raw));
            state[i] = (float)RawToNorm(st.calib[i], raw);
        }
        vector<float> state_n = NormalizeState(state, st.state_mean, st.state_std);

        // ---- Read + normalize cameras (BGR -> RGB -> /255 -> MEAN_STD, CHW) ----
        for (int ci = 0; ci < n_cam; ++ci) {
            cv::Mat frame;
            caps[ci] >> frame;
            if (frame.empty()) {
                cerr << "empty frame from " << st.cam_names[ci] << "\n";
                continue;
            }
            if (frame.cols != W || frame.rows != H) cv::resize(frame, frame, cv::Size(W, H));
            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);  // lerobot OpenCVCamera default RGB
            // HWC uint8 -> HWC float [0,1]
            const uint8_t* p = rgb.data;
            for (size_t k = 0; k < size_t(H) * W * 3; ++k) img_rgb[k] = p[k] / 255.0f;
            const auto& name = st.cam_names[ci];
            NormalizeImageInto(img_rgb, H, W, st.image_mean.at(name), st.image_std.at(name),
                images, size_t(ci) * 3 * H * W);
        }

        // ---- Predict a chunk only when the queue is empty ----
        if (q_head >= queue.size()) {
            queue.clear();
            q_head = 0;
            int chunk = 0, adim = 0;
            LocalTimer t;
            t.Start();
            auto raw = ort.RunChunk(images, img_shape, state_n, st_shape, chunk, adim);
            infer_ms.push_back(t.ElapsedMs());
            for (int s = 0; s < std::min(n_steps, chunk); ++s) {
                vector<float> a(adim);
                UnnormalizeAction(raw.data() + size_t(s) * adim, st.action_mean, st.action_std, a);
                queue.push_back(std::move(a));
            }
        }

        // ---- Pop one action, convert to raw step -> rad, command motors ----
        const vector<float>& action = queue[q_head++];
        vector<struct motor_cmd> cmds(S);
        for (int i = 0; i < S; ++i) {
            int raw = NormToRaw(st.calib[i], action[i], !cfg.no_clamp);
            cmds[i].mode = MOTOR_MODE_POS;
            cmds[i].pos_des = (float)(double(raw) / kMaxStep * kTwoPi);  // raw step -> rad
            cmds[i].vel_des = cfg.motor_speed;
            cmds[i].trq_des = 0.0f;
            cmds[i].kp = 0.0f;
            cmds[i].kd = 0.0f;
        }
        if (!cfg.dry_run) motor_set_cmds(motors.data(), cmds.data(), S);

        if (cfg.verbose) {
            cout << "[step " << step << "] state=[";
            for (int i = 0; i < S; ++i) cout << state[i] << (i + 1 < S ? "," : "]");
            cout << " action=[";
            for (int i = 0; i < A; ++i) cout << action[i] << (i + 1 < A ? "," : "]");
            cout << "\n";
        }

        double dt = duration_cast<duration<double>>(steady_clock::now() - loop_t0).count();
        loop_ms.push_back(dt * 1000.0);
        ++step;
        double sleep_s = 1.0 / cfg.fps - dt;
        if (sleep_s > 0) this_thread::sleep_for(duration<double>(sleep_s));
    }

    if (g_stop) cout << "\n[robot] interrupted; releasing hardware...\n";

    // Always release hardware, whether the loop ended on time, on error, or on
    // Ctrl+C. Ignore further termination signals during teardown so a second
    // Ctrl+C cannot interrupt motor idle/free or camera release.
    IgnoreSignalHandlers();
    vector<struct motor_cmd> idle(S);
    for (auto& c : idle) c.mode = MOTOR_MODE_IDLE;
    motor_set_cmds(motors.data(), idle.data(), S);
    motor_free(motors.data(), S);
    release_cameras();

    auto report = [](const char* tag, vector<double>& v) {
        if (v.empty()) return;
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (double x : v) sum += x;
        cout << "  " << tag << ": n=" << v.size() << " mean=" << sum / v.size()
            << "ms median=" << v[v.size() / 2] << "ms min=" << v.front()
            << "ms max=" << v.back() << "ms\n";
    };
    cout << "[robot] done: " << step << " steps\n";
    report("inference(chunk)", infer_ms);
    report("loop", loop_ms);
    return 0;
}
#endif  // ACT_ROBOT_HW

// --------------------------------------------------------------------------- //
int main(int argc, char** argv) {
    InstallSignalHandlers();
    Config cfg;
    if (!ParseArgs(argc, argv, cfg)) return 1;

    Stats st;
    try {
        {
            auto shared = ParseActStats(cfg.stats_path);
            st.cam_names = shared.cam_names;
            st.img_c = shared.img_c;
            st.img_h = shared.img_h;
            st.img_w = shared.img_w;
            st.state_dim = shared.state_dim;
            st.action_dim = shared.action_dim;
            st.chunk_size = shared.chunk_size;
            st.n_action_steps = shared.n_action_steps;
            st.motor_order = shared.motor_order;
            st.state_mean = shared.state_mean;
            st.state_std = shared.state_std;
            st.action_mean = shared.action_mean;
            st.action_std = shared.action_std;
            st.image_mean = shared.image_mean;
            st.image_std = shared.image_std;
            st.calib = shared.calib;
        }
    } catch (const std::exception& e) {
        cerr << "failed to parse stats: " << e.what() << "\n";
        return 1;
    }
    cout << "[act_evaluate] model=" << cfg.model_path
        << " provider=" << (cfg.use_ep ? "SpaceMIT EP" : "CPU")
        << " cams=" << st.cam_names.size() << " state_dim=" << st.state_dim
        << " action_dim=" << st.action_dim << " chunk=" << st.chunk_size << "\n";

    OnnxRunner ort(cfg);
    cout << "[act_evaluate] ONNX inputs=";
    for (auto& n : ort.in_names) cout << n << " ";
    cout << "outputs=";
    for (auto& n : ort.out_names) cout << n << " ";
    cout << "\n";

#ifdef ACT_ROBOT_HW
    if (!cfg.images_npy.empty() && !cfg.state_npy.empty())
        return RunOffline(cfg, st, ort);
    return RunRobot(cfg, st, ort);
#else
    return RunOffline(cfg, st, ort);
#endif
}

// NOLINTEND
