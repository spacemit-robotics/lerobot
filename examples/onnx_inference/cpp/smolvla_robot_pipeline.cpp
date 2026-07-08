// SmolVLA 4-model ONNX C++ real-robot test entry for SO-101.
//
// This is the hardware twin of smolvla_benchmark.cpp: it runs
// vision_encoder -> connector -> prefill_lm -> denoise_step and streams the
// unnormalized first 6 action dimensions to an SO-101 follower arm.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onnxruntime_cxx_api.h"
#include "utils/act_stats.h"

#ifdef ACT_ROBOT_HW
#include <opencv2/opencv.hpp>
extern "C" {
#include "motor.h"
}
#endif

using std::cerr;
using std::condition_variable;
using std::copy;
using std::cout;
using std::exception;
using std::fill;
using std::fixed;
using std::function;
using std::ifstream;
using std::invalid_argument;
using std::istringstream;
using std::make_unique;
using std::map;
using std::max;
using std::min;
using std::move;
using std::mutex;
using std::runtime_error;
using std::setprecision;
using std::setw;
using std::stoi;
using std::string;
using std::thread;
using std::this_thread::sleep_for;
using std::to_string;
using std::unique_lock;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;

namespace {

constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kMaxStep = 4095.0;

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }
void install_signal_handlers() {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

template <typename T>
size_t numel(const vector<T>& shape) {
    size_t n = 1;
    for (auto d : shape) {
        if (d <= 0) throw runtime_error("dynamic/non-positive tensor shape is not supported");
        n *= static_cast<size_t>(d);
    }
    return n;
}

vector<int64_t> shape_of(const Ort::Value& value) {
    auto s = value.GetTensorTypeAndShapeInfo().GetShape();
    for (auto& d : s) if (d < 0) d = 0;
    return s;
}

void set_env_default(const char* name, const char* value) {
    if (getenv(name) == nullptr) setenv(name, value, 0);
}

void set_spacemit_ep_env_defaults() {
    if (getenv("SMOLVLA_SKIP_EP_ENV_DEFAULTS") != nullptr) return;
    set_env_default("SPACEMIT_EP_PWCONV_INT8_USE", "1");
    set_env_default("SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_USE", "1");
    set_env_default("SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_LOG", "1");
    set_env_default("SPACEMIT_EP_CONVTRANSPOSE_3X3_FP16_USE", "1");
    set_env_default("ORT_CPU_EP_DIV_FP32_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_CONCAT_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_GATHER_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_MUL_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_ERF_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_SIN_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_COS_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_WHERE_FP16_USE", "1");
    set_env_default("SPACEMIT_EP_POW_FP16_RVV_USE", "0");
    set_env_default("SPACEMIT_EP_REDUCEMEAN_FP16_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_SOFTMAX_FP16_USE", "0");
    set_env_default("SPACEMIT_EP_CONV3D_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_SOFTMAX_FP32_USE", "0");
    set_env_default("SPACEMIT_EP_REDUCESUM_FP32_USE", "1");
    set_env_default("SPACEMIT_EP_SEPDWCONV_USE", "1");
    set_env_default("SPACEMIT_EP_DWCONV_3X3_FP32_USE", "1");
    set_env_default("SPACEMIT_EP_DWCONV_3X3_S2_FP32_USE", "1");
    set_env_default("SPACEMIT_EP_ADD_QDQ_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_REDUCEMEAN_QDQ_RVV_USE", "1");
    set_env_default("SPACEMIT_EP_SOFTMAX_QDQ_INT8_USE", "1");
    set_env_default("SPACEMIT_EP_GELU_QDQ_INT8_USE", "1");
    set_env_default("SPACEMIT_EP_LAYERNORM_QDQ_INT8_USE", "1");
    set_env_default("SPACEMIT_EP_CONVTRANSPOSE_3X3_USE", "1");
    set_env_default("SPACEMIT_EP_CONVTRANSPOSE_4X4_USE", "1");
    set_env_default("SPACEMIT_EP_POW2_REDUCEMEAN_USE", "0");
}

double raw_to_norm(const MotorCalib& c, int raw) {
    double out;
    if (c.norm_mode == "RANGE_0_100") {
        double span = static_cast<double>(c.range_max - c.range_min);
        out = span != 0.0 ? (static_cast<double>(raw - c.range_min) / span) * 100.0 : 0.0;
        if (c.drive_mode == 1) out = 100.0 - out;
    } else {
        double mid = (c.range_min + c.range_max) / 2.0;
        out = (static_cast<double>(raw) - mid) * 360.0 / kMaxStep;
        if (c.drive_mode == 1) out = -out;
    }
    return out;
}

int norm_to_raw(const MotorCalib& c, double val, bool clamp_to_range) {
    double raw;
    if (c.norm_mode == "RANGE_0_100") {
        if (c.drive_mode == 1) val = 100.0 - val;
        val = min(100.0, max(0.0, val));
        raw = (val / 100.0) * static_cast<double>(c.range_max - c.range_min) + c.range_min;
    } else {
        if (c.drive_mode == 1) val = -val;
        double mid = (c.range_min + c.range_max) / 2.0;
        raw = val * kMaxStep / 360.0 + mid;
    }
    int iraw = static_cast<int>(llround(raw));
    if (clamp_to_range) iraw = min(c.range_max, max(c.range_min, iraw));
    return min(4095, max(0, iraw));
}

struct RuntimeMeta {
    string task;
    vector<string> image_keys;
    int resize_w = 512;
    int resize_h = 512;
    int tokenizer_max_length = 48;
    int state_dim = 6;
    int action_dim = 6;
    int max_state_dim = 32;
    int max_action_dim = 32;
    int chunk_size = 50;
    int n_action_steps = 50;
    int empty_cameras = 0;
    vector<string> motor_order;
    vector<float> state_mean, state_std, action_mean, action_std;
    vector<int64_t> lang_tokens, lang_masks;
    vector<MotorCalib> calib;
};

vector<int64_t> parse_i64s(istringstream& ss) {
    vector<int64_t> v;
    int64_t x;
    while (ss >> x) v.push_back(x);
    return v;
}

RuntimeMeta parse_runtime_meta(const string& path) {
    ifstream f(path);
    if (!f) throw runtime_error("cannot open runtime metadata: " + path);
    RuntimeMeta m;
    map<string, MotorCalib> calib_by_name;
    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        istringstream ss(line);
        string key;
        ss >> key;
        if (key == "task") {
            string rest;
            getline(ss, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(rest.begin());
            m.task = rest;
        } else if (key == "image_keys") {
            string x;
            while (ss >> x) m.image_keys.push_back(x);
        } else if (key == "resize_hw") {
            ss >> m.resize_w >> m.resize_h;
        } else if (key == "tokenizer_max_length") ss >> m.tokenizer_max_length;
        else if (key == "state_dim") ss >> m.state_dim;
        else if (key == "action_dim") ss >> m.action_dim;
        else if (key == "max_state_dim") ss >> m.max_state_dim;
        else if (key == "max_action_dim") ss >> m.max_action_dim;
        else if (key == "chunk_size") ss >> m.chunk_size;
        else if (key == "n_action_steps") ss >> m.n_action_steps;
        else if (key == "empty_cameras") ss >> m.empty_cameras;
        else if (key == "motor_order") {
            string x;
            while (ss >> x) m.motor_order.push_back(x);
        } else if (key == "state_mean") m.state_mean = ParseFloats(ss);
        else if (key == "state_std") m.state_std = ParseFloats(ss);
        else if (key == "action_mean") m.action_mean = ParseFloats(ss);
        else if (key == "action_std") m.action_std = ParseFloats(ss);
        else if (key == "lang_tokens") m.lang_tokens = parse_i64s(ss);
        else if (key == "lang_masks") m.lang_masks = parse_i64s(ss);
        else if (key == "calib") {
            MotorCalib c;
            ss >> c.name;
            string tok;
            while (ss >> tok) {
                if (tok == "id") ss >> c.id;
                else if (tok == "drive_mode") ss >> c.drive_mode;
                else if (tok == "range_min") ss >> c.range_min;
                else if (tok == "range_max") ss >> c.range_max;
                else if (tok == "norm_mode") ss >> c.norm_mode;
            }
            calib_by_name[c.name] = c;
        }
    }
    for (const auto& motor : m.motor_order) {
        auto it = calib_by_name.find(motor);
        if (it != calib_by_name.end()) m.calib.push_back(it->second);
    }
    if (static_cast<int>(m.state_mean.size()) != m.state_dim ||
        static_cast<int>(m.action_mean.size()) != m.action_dim ||
        static_cast<int>(m.lang_tokens.size()) != m.tokenizer_max_length ||
        static_cast<int>(m.lang_masks.size()) != m.tokenizer_max_length) {
        throw runtime_error("runtime metadata is incomplete or inconsistent");
    }
    return m;
}

string image_key_to_camera_name(const string& image_key) {
    const string prefix = "observation.images.";
    if (image_key.rfind(prefix, 0) == 0) return image_key.substr(prefix.size());
    const size_t pos = image_key.find_last_of('.');
    return pos == string::npos ? image_key : image_key.substr(pos + 1);
}

map<string, string> legacy_camera_aliases() {
    return {
        {"camera1", "top"},
        {"camera2", "wrist"},
        {"top", "camera1"},
        {"wrist", "camera2"},
    };
}

struct Args {
    string model_dir = "../models/onnx/smolvla-fast-4model-2cam-fp16-prefill-split";
    string runtime = "../models/onnx/smolvla_runtime.txt";
    string port = "/dev/ttyACM0";
    uint32_t baud = 1000000;
    map<string, int> cam_index{{"top", 15}, {"wrist", 13}};
    double fps = 30.0;
    double episode_time = 60.0;
    int max_iters = 0;
    int denoise_steps = 10;
    int n_action_steps = 0;
    int warmup = 0;
    uint32_t seed = 0;
    bool use_spacemit_ep = true;
    bool prefill_ep = true;
    bool prefill_iobind = true;
    bool vision_ep = true;
    bool connector_ep = true;
    bool denoise_ep = true;
    int ep_threads = 8;
    string ep_affinity = "8;9;10;11;12;13;14;15";
    bool dry_run = false;
    bool no_motors = false;
    bool no_clamp = false;
    bool print_actions = false;
    bool debug_numerics = false;
    bool disable_graph_opt = false;
    bool global_ep_pool = false;
    bool per_camera_vision = false;
    bool warmup_only = false;
    bool infer_every_tick = false;
    bool cpu_shadow_correct = false;
    float shadow_action_tol = 1.0f;
    float motor_speed = 3.0f;
};

string require_value(int& i, int argc, char* argv[], const string& flag) {
    if (i + 1 >= argc) throw invalid_argument(flag + " requires a value");
    return argv[++i];
}

void print_usage(const char* prog) {
    cout << "Usage: " << prog << " [options]\n"
        << "  --model-dir DIR       ONNX model directory\n"
        << "  --runtime FILE        SmolVLA runtime metadata from export_smolvla_runtime.py\n"
        << "  --port P              SO-101 serial port (default /dev/ttyACM0)\n"
        << "  --camera NAME=IDX     Camera mapping, e.g. --camera top=15 --camera wrist=13\n"
        << "  --fps F               Control loop target FPS (default 30)\n"
        << "  --episode-time S      Seconds to run (default 60)\n"
        << "  --max-iters N         Hard loop cap\n"
        << "  --denoise-steps N     Denoise steps per chunk (default 10)\n"
        << "  --n-action-steps N    Actions to enqueue from each inferred chunk (default runtime metadata)\n"
        << "  --seed N              Denoise noise seed shared with the Python runner (default 0)\n"
        << "  --warmup N            Synthetic warmup chunks before hardware loop\n"
        << "  --warmup-only         Run synthetic warmup and exit before hardware loop\n"
        << "  --use-spacemit-ep     Enable SpaceMIT EP (default)\n"
        << "  --cpu                 CPU only\n"
        << "  --prefill-ep          Run prefill on EP (default with --use-spacemit-ep)\n"
        << "  --cpu-prefill         Run prefill on CPU even when --use-spacemit-ep is enabled\n"
        << "  --no-prefill-iobind   Run prefill EP session through plain Session.Run for diagnostics\n"
        << "  --denoise-ep          Run denoise on EP (default with --use-spacemit-ep)\n"
        << "  --cpu-denoise         Run denoise on CPU even when --use-spacemit-ep is enabled\n"
        << "  --cpu-vision          Run vision_encoder on CPU\n"
        << "  --cpu-connector       Run connector on CPU\n"
        << "  --ep-threads N        SpaceMIT EP thread count\n"
        << "  --ep-affinity LIST    SpaceMIT EP affinity\n"
        << "  --global-ep-pool      Use one global shared SpaceMIT EP thread pool\n"
        << "  --per-camera-vision   Run vision/connector once per camera like the Python evaluator\n"
        << "  --dry-run             Compute actions but do not command motors\n"
        << "  --no-motors           Do not open motors; use zero normalized state for diagnostics\n"
        << "  --cpu-shadow-correct  Run all requested EP sessions, then correct action with CPU shadow if EP diverges\n"
        << "  --shadow-action-tol X Max allowed unnormalized action diff before CPU correction (default 1)\n"
        << "  --print-actions       Print each action sent\n"
        << "  --infer-every-tick    Recompute a full action chunk every loop iteration\n"
        << "  --debug-numerics      Print tensor finite/min/max summaries\n"
        << "  --disable-graph-opt   Disable ORT graph optimization\n";
}

Args parse_args(int argc, char* argv[]) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--model-dir") a.model_dir = require_value(i, argc, argv, arg);
        else if (arg == "--runtime") a.runtime = require_value(i, argc, argv, arg);
        else if (arg == "--port") a.port = require_value(i, argc, argv, arg);
        else if (arg == "--baud") a.baud = static_cast<uint32_t>(stoul(require_value(i, argc, argv, arg)));
        else if (arg == "--camera") {
            string kv = require_value(i, argc, argv, arg);
            auto pos = kv.find('=');
            if (pos == string::npos) throw invalid_argument("--camera expects NAME=IDX");
            a.cam_index[kv.substr(0, pos)] = stoi(kv.substr(pos + 1));
        } else if (arg == "--fps") a.fps = stod(require_value(i, argc, argv, arg));
        else if (arg == "--episode-time") a.episode_time = stod(require_value(i, argc, argv, arg));
        else if (arg == "--max-iters") a.max_iters = stoi(require_value(i, argc, argv, arg));
        else if (arg == "--denoise-steps") a.denoise_steps = stoi(require_value(i, argc, argv, arg));
        else if (arg == "--n-action-steps") a.n_action_steps = stoi(require_value(i, argc, argv, arg));
        else if (arg == "--warmup") a.warmup = stoi(require_value(i, argc, argv, arg));
        else if (arg == "--warmup-only") a.warmup_only = true;
        else if (arg == "--seed") a.seed = static_cast<uint32_t>(stoul(require_value(i, argc, argv, arg)));
        else if (arg == "--use-spacemit-ep" || arg == "-s") a.use_spacemit_ep = true;
        else if (arg == "--cpu") a.use_spacemit_ep = false;
        else if (arg == "--prefill-ep") a.prefill_ep = true;
        else if (arg == "--cpu-prefill") a.prefill_ep = false;
        else if (arg == "--no-prefill-iobind") a.prefill_iobind = false;
        else if (arg == "--denoise-ep") a.denoise_ep = true;
        else if (arg == "--cpu-denoise") a.denoise_ep = false;
        else if (arg == "--cpu-vision") a.vision_ep = false;
        else if (arg == "--cpu-connector") a.connector_ep = false;
        else if (arg == "--ep-threads") a.ep_threads = stoi(require_value(i, argc, argv, arg));
        else if (arg == "--ep-affinity") a.ep_affinity = require_value(i, argc, argv, arg);
        else if (arg == "--global-ep-pool") a.global_ep_pool = true;
        else if (arg == "--per-camera-vision") a.per_camera_vision = true;
        else if (arg == "--dry-run") a.dry_run = true;
        else if (arg == "--no-motors") a.no_motors = true;
        else if (arg == "--cpu-shadow-correct") a.cpu_shadow_correct = true;
        else if (arg == "--shadow-action-tol") a.shadow_action_tol = stof(require_value(i, argc, argv, arg));
        else if (arg == "--no-clamp") a.no_clamp = true;
        else if (arg == "--print-actions") a.print_actions = true;
        else if (arg == "--infer-every-tick") a.infer_every_tick = true;
        else if (arg == "--debug-numerics") a.debug_numerics = true;
        else if (arg == "--disable-graph-opt") a.disable_graph_opt = true;
        else if (arg == "--motor-speed") a.motor_speed = stof(require_value(i, argc, argv, arg));
        else if (arg == "--help" || arg == "-h") { print_usage(argv[0]); exit(0); }
        else throw invalid_argument("unknown arg: " + arg);
    }
    if (a.denoise_steps <= 0) throw invalid_argument("--denoise-steps must be positive");
    if (a.n_action_steps < 0) throw invalid_argument("--n-action-steps must be positive");
    return a;
}

void add_provider_option_from_env(unordered_map<string, string>& options, const char* name) {
    const char* value = getenv(name);
    if (value != nullptr && value[0] != '\0') options[name] = value;
}

Ort::SessionOptions make_opts(const Args& args, bool ep) {
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(
        args.disable_graph_opt ? GraphOptimizationLevel::ORT_DISABLE_ALL : GraphOptimizationLevel::ORT_ENABLE_ALL);
    opts.SetIntraOpNumThreads(args.ep_threads);
    if (ep && args.use_spacemit_ep) {
        unordered_map<string, string> po;
        if (args.global_ep_pool) po["SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD"] = "1";
        po["SPACEMIT_EP_INTRA_THREAD_NUM"] = to_string(args.ep_threads);
        if (!args.ep_affinity.empty()) po["SPACEMIT_EP_INTRA_THREAD_AFFINITY"] = args.ep_affinity;
        add_provider_option_from_env(po, "SPACEMIT_EP_DISABLE_OP_TYPE_FILTER");
        add_provider_option_from_env(po, "SPACEMIT_EP_DISABLE_OP_NAME_FILTER");
        add_provider_option_from_env(po, "SPACEMIT_EP_DISABLE_FLOAT16_EPILOGUE");
        add_provider_option_from_env(po, "SPACEMIT_EP_DUMP_SUBGRAPHS");
        add_provider_option_from_env(po, "SPACEMIT_EP_DEBUG_PROFILE");
        add_provider_option_from_env(po, "SPACEMIT_EP_DUMP_TENSORS");
        void* ep_handle = nullptr;
        if (!ep_handle) ep_handle = dlopen("libspacemit_ep.so", RTLD_NOW);
        if (!ep_handle) ep_handle = dlopen("libspacemit_ep.so.2", RTLD_NOW);
        if (!ep_handle) throw runtime_error("failed to dlopen libspacemit_ep.so");
        using EpInitFn = OrtStatus* (*)(OrtSessionOptions*, const char* const*, const char* const*, size_t);
        auto ep_init = reinterpret_cast<EpInitFn>(dlsym(ep_handle, "OrtSessionOptionsSpaceMITEnvInit"));
        if (!ep_init) throw runtime_error("failed to dlsym OrtSessionOptionsSpaceMITEnvInit");
        vector<string> keys_storage;
        vector<string> vals_storage;
        vector<const char*> keys;
        vector<const char*> vals;
        keys_storage.reserve(po.size());
        vals_storage.reserve(po.size());
        keys.reserve(po.size());
        vals.reserve(po.size());
        for (const auto& kv : po) {
            keys_storage.push_back(kv.first);
            vals_storage.push_back(kv.second);
        }
        for (size_t i = 0; i < keys_storage.size(); ++i) {
            keys.push_back(keys_storage[i].c_str());
            vals.push_back(vals_storage[i].c_str());
        }
        OrtStatus* st = ep_init(opts, keys.data(), vals.data(), keys.size());
        if (st != nullptr) throw runtime_error("SpaceMIT EP init failed");
    }
    return opts;
}

class SessionWrap {
public:
    SessionWrap(Ort::Env& env, const string& path, Ort::SessionOptions opts)
        : path_(path) {
        thread_ = thread([this, &env, o = move(opts)]() mutable {
            session_ = make_unique<Ort::Session>(env, path_.c_str(), o);
            Ort::AllocatorWithDefaultOptions alloc;
            for (size_t i = 0; i < session_->GetInputCount(); ++i) {
                input_names_.emplace_back(session_->GetInputNameAllocated(i, alloc).get());
                input_shapes_.push_back(session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }
            for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
                output_names_.emplace_back(session_->GetOutputNameAllocated(i, alloc).get());
                output_shapes_.push_back(session_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            }
            for (auto& s : input_names_) input_ptrs_.push_back(s.c_str());
            for (auto& s : output_names_) output_ptrs_.push_back(s.c_str());

            {
                unique_lock<mutex> lk(mtx_);
                ready_ = true;
            }
            ready_cv_.notify_one();

            while (true) {
                function<void()> task;
                {
                    unique_lock<mutex> lk(mtx_);
                    work_cv_.wait(lk, [this] { return has_task_ || stop_; });
                    if (stop_) break;
                    task = move(task_);
                    has_task_ = false;
                }
                task();
                {
                    unique_lock<mutex> lk(mtx_);
                    done_ = true;
                }
                done_cv_.notify_one();
            }
        });
        unique_lock<mutex> lk(mtx_);
        ready_cv_.wait(lk, [this] { return ready_; });
    }

    ~SessionWrap() { stop(); }

    vector<Ort::Value> run(vector<Ort::Value> inputs, bool iobind = false) {
        vector<Ort::Value> outputs;
        {
            unique_lock<mutex> lk(mtx_);
            task_ = [this, &inputs, &outputs, iobind]() {
                if (!iobind) {
                    outputs = session_->Run(
                        Ort::RunOptions{nullptr},
                        input_ptrs_.data(),
                        inputs.data(),
                        inputs.size(),
                        output_ptrs_.data(),
                        output_ptrs_.size());
                    return;
                }
                Ort::IoBinding binding(*session_);
                auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
                for (size_t i = 0; i < inputs.size(); ++i) binding.BindInput(input_ptrs_.at(i), inputs.at(i));
                for (const char* output : output_ptrs_) binding.BindOutput(output, mem);
                session_->Run(Ort::RunOptions{nullptr}, binding);
                outputs = binding.GetOutputValues();
            };
            has_task_ = true;
            done_ = false;
        }
        work_cv_.notify_one();
        unique_lock<mutex> lk(mtx_);
        done_cv_.wait(lk, [this] { return done_; });
        return outputs;
    }

    vector<int64_t> input_shape(size_t i) const { return input_shapes_.at(i); }
    vector<int64_t> output_shape(size_t i) const { return output_shapes_.at(i); }

private:
    void stop() {
        if (!thread_.joinable()) return;
        {
            unique_lock<mutex> lk(mtx_);
            stop_ = true;
        }
        work_cv_.notify_one();
        thread_.join();
    }

    string path_;
    unique_ptr<Ort::Session> session_;
    vector<string> input_names_, output_names_;
    vector<vector<int64_t>> input_shapes_, output_shapes_;
    vector<const char*> input_ptrs_, output_ptrs_;
    thread thread_;
    mutable mutex mtx_;
    condition_variable work_cv_, done_cv_, ready_cv_;
    function<void()> task_;
    bool has_task_ = false;
    bool done_ = false;
    bool stop_ = false;
    bool ready_ = false;
};

struct PipelineResult {
    vector<float> actions;
    double vision_ms = 0;
    double connector_ms = 0;
    double prefill_ms = 0;
    double denoise_ms = 0;
};

bool actions_finite(const vector<float>& actions) {
    for (float value : actions) {
        if (!isfinite(value)) return false;
    }
    return true;
}

void print_tensor_stats(const string& name, const float* data, size_t n) {
    size_t bad = 0;
    double sum = 0.0;
    float min_v = INFINITY;
    float max_v = -INFINITY;
    for (size_t i = 0; i < n; ++i) {
        float v = data[i];
        if (!isfinite(v)) {
            ++bad;
            continue;
        }
        min_v = min(min_v, v);
        max_v = max(max_v, v);
        sum += v;
    }
    const size_t good = n - bad;
    cout << "[num] " << name
        << " n=" << n
        << " bad=" << bad;
    if (good > 0) {
        cout << " min=" << fixed << setprecision(6) << min_v
            << " max=" << max_v
            << " mean=" << (sum / static_cast<double>(good));
    }
    cout << "\n";
}

void print_bool_stats(const string& name, const bool* data, size_t n) {
    size_t true_count = 0;
    for (size_t i = 0; i < n; ++i) if (data[i]) ++true_count;
    cout << "[num] " << name
        << " n=" << n
        << " true=" << true_count
        << " false=" << (n - true_count) << "\n";
}

float max_abs_diff(const vector<float>& a, const vector<float>& b) {
    if (a.size() != b.size()) return INFINITY;
    float out = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!isfinite(a[i]) || !isfinite(b[i])) return INFINITY;
        out = max(out, fabsf(a[i] - b[i]));
    }
    return out;
}

class PortableNormalRng {
public:
    explicit PortableNormalRng(uint64_t seed)
        : state_(seed) {}

    float next_normal() {
        if (has_spare_) {
            has_spare_ = false;
            return static_cast<float>(spare_);
        }
        const double u1 = uniform_open();
        const double u2 = uniform_open();
        const double radius = sqrt(-2.0 * log(u1));
        const double theta = kTwoPi * u2;
        spare_ = radius * sin(theta);
        has_spare_ = true;
        return static_cast<float>(radius * cos(theta));
    }

private:
    uint64_t next_u64() {
        state_ += UINT64_C(0x9E3779B97F4A7C15);
        uint64_t value = state_;
        value = (value ^ (value >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
        value = (value ^ (value >> 27)) * UINT64_C(0x94D049BB133111EB);
        return value ^ (value >> 31);
    }

    double uniform_open() {
        constexpr double kInv53 = 1.0 / static_cast<double>(UINT64_C(1) << 53);
        return static_cast<double>((next_u64() >> 11) + 0.5) * kInv53;
    }

    uint64_t state_ = 0;
    bool has_spare_ = false;
    double spare_ = 0.0;
};

class SmolVLAPipeline {
public:
    SmolVLAPipeline(const Args& args, const RuntimeMeta& meta)
        : args_(args),
        meta_(meta),
        env_(ORT_LOGGING_LEVEL_WARNING, "smolvla_evaluate"),
        mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
        vision_(env_, args.model_dir + "/vision_encoder.onnx", make_opts(args, args.vision_ep)),
        connector_(env_, args.model_dir + "/connector.onnx", make_opts(args, args.connector_ep)),
        prefill_(env_, args.model_dir + "/prefill_lm.onnx", make_opts(args, args.prefill_ep)),
        denoise_(env_, args.model_dir + "/denoise_step.onnx", make_opts(args, args.denoise_ep)),
        rng_(args.seed) {
        auto vision_shape = vision_.input_shape(0);
        auto prefill_shape = prefill_.input_shape(0);
        expected_image_tokens_ = prefill_shape.at(1);
        if (vision_shape.at(0) > 1 && !args.per_camera_vision) {
            camera_count_ = static_cast<int>(vision_shape.at(0));
            batched_vision_ = true;
        } else if (vision_shape.at(0) <= 0) {
            auto connector_shape = connector_.output_shape(0);
            int64_t tokens_per_camera = connector_shape.size() > 1 && connector_shape[1] > 0
                ? connector_shape[1]
                : 64;
            if (expected_image_tokens_ % tokens_per_camera != 0) {
                throw runtime_error("cannot derive camera count from connector/prefill shapes");
            }
            camera_count_ = static_cast<int>(expected_image_tokens_ / tokens_per_camera);
            batched_vision_ = !args.per_camera_vision;
        } else {
            auto connector_shape = connector_.output_shape(0);
            int64_t tokens_per_camera = connector_shape.size() > 1 && connector_shape[1] > 0
                ? connector_shape[1]
                : 64;
            if (expected_image_tokens_ % tokens_per_camera != 0) {
                throw runtime_error("cannot derive camera count from connector/prefill shapes");
            }
            camera_count_ = static_cast<int>(expected_image_tokens_ / tokens_per_camera);
            batched_vision_ = false;
        }
        if (args.per_camera_vision && vision_shape.at(0) > 1) {
            throw runtime_error("--per-camera-vision requires a dynamic or batch-1 vision input");
        }
        cout << "[smolvla] camera_count=" << camera_count_
            << " expected_image_tokens=" << expected_image_tokens_
            << " vision_mode=" << (batched_vision_ ? "batched" : "per-camera")
            << " vision=" << (args.vision_ep && args.use_spacemit_ep ? "EP" : "CPU")
            << " connector=" << (args.connector_ep && args.use_spacemit_ep ? "EP" : "CPU")
            << " prefill=" << (args.prefill_ep && args.use_spacemit_ep ? "EP" : "CPU")
            << " denoise=" << (args.denoise_ep && args.use_spacemit_ep ? "EP" : "CPU")
            << "\n";
    }

    int camera_count() const { return camera_count_; }

    PipelineResult run(const vector<float>& images, const vector<float>& state_norm_padded) {
        PipelineResult result;
        auto t0 = steady_clock::now();
        vector<float> image_embs_storage;
        vector<int64_t> embs_shape;

        if (batched_vision_) {
            vector<int64_t> image_shape = vision_.input_shape(0);
            image_shape[0] = camera_count_;
            vector<Ort::Value> vin;
            vin.push_back(Ort::Value::CreateTensor<float>(
                mem_, const_cast<float*>(images.data()), images.size(), image_shape.data(), image_shape.size()));
            auto vout = vision_.run(move(vin));
            result.vision_ms = elapsed_ms(t0);
            if (args_.debug_numerics) {
                auto vshape = shape_of(vout[0]);
                auto* vptr = vout[0].GetTensorMutableData<float>();
                print_tensor_stats("vision.hidden.batched", vptr, numel(vshape));
            }

            t0 = steady_clock::now();
            vector<Ort::Value> cin;
            cin.push_back(move(vout[0]));
            auto cout_v = connector_.run(move(cin));
            result.connector_ms = elapsed_ms(t0);
            embs_shape = shape_of(cout_v[0]);
            auto* ptr = cout_v[0].GetTensorMutableData<float>();
            if (args_.debug_numerics) print_tensor_stats("connector.image_embs.batched", ptr, numel(embs_shape));
            image_embs_storage.assign(ptr, ptr + numel(embs_shape));
            if (embs_shape.size() == 3 && embs_shape[0] == camera_count_) {
                embs_shape = {1, embs_shape[0] * embs_shape[1], embs_shape[2]};
            }
        } else {
            vector<int64_t> image_shape = vision_.input_shape(0);
            image_shape[0] = 1;
            const size_t per_image = numel(image_shape);
            vector<vector<float>> parts;
            for (int cam = 0; cam < camera_count_; ++cam) {
                t0 = steady_clock::now();
                vector<Ort::Value> vin;
                vin.push_back(Ort::Value::CreateTensor<float>(
                    mem_,
                    const_cast<float*>(images.data() + static_cast<size_t>(cam) * per_image),
                    per_image,
                    image_shape.data(),
                    image_shape.size()));
                auto vout = vision_.run(move(vin));
                result.vision_ms += elapsed_ms(t0);
                if (args_.debug_numerics) {
                    auto vshape = shape_of(vout[0]);
                    auto* vptr = vout[0].GetTensorMutableData<float>();
                    print_tensor_stats("vision.hidden[" + to_string(cam) + "]", vptr, numel(vshape));
                }

                t0 = steady_clock::now();
                vector<Ort::Value> cin;
                cin.push_back(move(vout[0]));
                auto cout_v = connector_.run(move(cin));
                result.connector_ms += elapsed_ms(t0);
                auto cshape = shape_of(cout_v[0]);
                if (embs_shape.empty()) embs_shape = {1, 0, cshape.at(2)};
                auto* ptr = cout_v[0].GetTensorMutableData<float>();
                if (args_.debug_numerics) {
                    print_tensor_stats("connector.image_embs[" + to_string(cam) + "]", ptr, numel(cshape));
                }
                parts.emplace_back(ptr, ptr + numel(cshape));
                embs_shape[1] += cshape.at(1);
            }
            image_embs_storage.reserve(numel(embs_shape));
            for (auto& part : parts) image_embs_storage.insert(image_embs_storage.end(), part.begin(), part.end());
        }

        if (embs_shape.at(1) != expected_image_tokens_) {
            throw runtime_error("connector output token count does not match prefill input");
        }
        float* image_embs = image_embs_storage.data();
        if (args_.debug_numerics) {
            print_tensor_stats("image_embs", image_embs, image_embs_storage.size());
            print_tensor_stats("state_norm_padded", state_norm_padded.data(), state_norm_padded.size());
        }

        t0 = steady_clock::now();
        vector<int64_t> lang_shape = {1, meta_.tokenizer_max_length};
        vector<int64_t> state_shape = {1, meta_.max_state_dim};
        vector<Ort::Value> pin;
        pin.push_back(Ort::Value::CreateTensor<float>(
            mem_, image_embs, numel(embs_shape), embs_shape.data(), embs_shape.size()));
        pin.push_back(Ort::Value::CreateTensor<int64_t>(
            mem_, const_cast<int64_t*>(meta_.lang_tokens.data()), meta_.lang_tokens.size(),
            lang_shape.data(), lang_shape.size()));
        pin.push_back(Ort::Value::CreateTensor<int64_t>(
            mem_, const_cast<int64_t*>(meta_.lang_masks.data()), meta_.lang_masks.size(),
            lang_shape.data(), lang_shape.size()));
        pin.push_back(Ort::Value::CreateTensor<float>(
            mem_, const_cast<float*>(state_norm_padded.data()), state_norm_padded.size(),
            state_shape.data(), state_shape.size()));
        auto pout = prefill_.run(move(pin), args_.prefill_ep && args_.prefill_iobind);
        result.prefill_ms = elapsed_ms(t0);

        vector<int64_t> mask_shape;
        vector<int64_t> kv_shape;
        vector<float> stacked_keys, stacked_values;
        float* past_keys = nullptr;
        float* past_values = nullptr;
        bool* prefix_masks = nullptr;
        if (pout.size() == 3) {
            kv_shape = shape_of(pout[0]);
            mask_shape = shape_of(pout[2]);
            past_keys = pout[0].GetTensorMutableData<float>();
            past_values = pout[1].GetTensorMutableData<float>();
            prefix_masks = pout[2].GetTensorMutableData<bool>();
            if (args_.debug_numerics) {
                print_tensor_stats("prefill.past_keys", past_keys, numel(kv_shape));
                print_tensor_stats("prefill.past_values", past_values, numel(kv_shape));
                print_bool_stats("prefill.prefix_masks", prefix_masks, numel(mask_shape));
            }
        } else if (pout.size() == 33) {
            auto layer_shape = shape_of(pout[0]);
            if (layer_shape.size() == 4) {
                kv_shape = {16, layer_shape[0], layer_shape[1], layer_shape[2], layer_shape[3]};
            } else if (layer_shape.size() == 5) {
                kv_shape = {16, layer_shape[1], layer_shape[2], layer_shape[3], layer_shape[4]};
            } else {
                throw runtime_error("unexpected split prefill KV rank");
            }
            const size_t layer_elems = numel(layer_shape);
            stacked_keys.resize(layer_elems * 16);
            stacked_values.resize(layer_elems * 16);
            for (size_t layer = 0; layer < 16; ++layer) {
                auto* k = pout[layer].GetTensorMutableData<float>();
                auto* v = pout[layer + 16].GetTensorMutableData<float>();
                if (args_.debug_numerics) {
                    print_tensor_stats("prefill.key[" + to_string(layer) + "]", k, layer_elems);
                    print_tensor_stats("prefill.value[" + to_string(layer) + "]", v, layer_elems);
                }
                copy(k, k + layer_elems, stacked_keys.begin() + layer * layer_elems);
                copy(v, v + layer_elems, stacked_values.begin() + layer * layer_elems);
            }
            mask_shape = shape_of(pout[32]);
            past_keys = stacked_keys.data();
            past_values = stacked_values.data();
            prefix_masks = pout[32].GetTensorMutableData<bool>();
            if (args_.debug_numerics) {
                print_bool_stats("prefill.prefix_masks", prefix_masks, numel(mask_shape));
                print_tensor_stats("prefill.stacked_keys", past_keys, stacked_keys.size());
                print_tensor_stats("prefill.stacked_values", past_values, stacked_values.size());
            }
        } else {
            throw runtime_error("unsupported prefill output count: " + to_string(pout.size()));
        }

        vector<int64_t> noise_shape = {1, meta_.chunk_size, meta_.max_action_dim};
        vector<float> x_t(numel(noise_shape));
        for (float& x : x_t) x = rng_.next_normal();
        if (args_.debug_numerics) print_tensor_stats("denoise.x_t.initial", x_t.data(), x_t.size());

        t0 = steady_clock::now();
        vector<int64_t> ts_shape = {1};
        const float dt = -1.0f / static_cast<float>(args_.denoise_steps);
        for (int step = 0; step < args_.denoise_steps; ++step) {
            float ts = 1.0f + static_cast<float>(step) * dt;
            vector<Ort::Value> din;
            din.push_back(Ort::Value::CreateTensor<float>(
                mem_, x_t.data(), x_t.size(), noise_shape.data(), noise_shape.size()));
            din.push_back(Ort::Value::CreateTensor<float>(
                mem_, &ts, 1, ts_shape.data(), ts_shape.size()));
            din.push_back(Ort::Value::CreateTensor<bool>(
                mem_, prefix_masks, numel(mask_shape), mask_shape.data(), mask_shape.size()));
            din.push_back(Ort::Value::CreateTensor<float>(
                mem_, past_keys, numel(kv_shape), kv_shape.data(), kv_shape.size()));
            din.push_back(Ort::Value::CreateTensor<float>(
                mem_, past_values, numel(kv_shape), kv_shape.data(), kv_shape.size()));
            auto denoise_outputs = denoise_.run(move(din));
            auto* v_t = denoise_outputs[0].GetTensorMutableData<float>();
            if (args_.debug_numerics) {
                print_tensor_stats("denoise.v_t[" + to_string(step) + "]", v_t, x_t.size());
            }
            for (size_t i = 0; i < x_t.size(); ++i) x_t[i] += dt * v_t[i];
            if (args_.debug_numerics) {
                print_tensor_stats("denoise.x_t[" + to_string(step) + "]", x_t.data(), x_t.size());
            }
        }
        result.denoise_ms = elapsed_ms(t0);
        if (args_.debug_numerics) print_tensor_stats("denoise.x_t.final", x_t.data(), x_t.size());

        result.actions.resize(static_cast<size_t>(meta_.chunk_size) * meta_.action_dim);
        for (int step = 0; step < meta_.chunk_size; ++step) {
            for (int j = 0; j < meta_.action_dim; ++j) {
                float z = x_t[static_cast<size_t>(step) * meta_.max_action_dim + j];
                result.actions[static_cast<size_t>(step) * meta_.action_dim + j] =
                    z * meta_.action_std[j] + meta_.action_mean[j];
            }
        }
        if (args_.debug_numerics) print_tensor_stats("actions.unnorm", result.actions.data(), result.actions.size());
        return result;
    }

private:
    static double elapsed_ms(steady_clock::time_point start) {
        return duration_cast<microseconds>(steady_clock::now() - start).count() / 1000.0;
    }

    const Args& args_;
    const RuntimeMeta& meta_;
    Ort::Env env_;
    Ort::MemoryInfo mem_;
    SessionWrap vision_, connector_, prefill_, denoise_;
    PortableNormalRng rng_;
    int camera_count_ = 1;
    int64_t expected_image_tokens_ = 0;
    bool batched_vision_ = false;
};

#ifdef ACT_ROBOT_HW
void preprocess_frame(const cv::Mat& bgr, vector<float>& out, size_t base, int resize_w, int resize_h) {
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    const int cur_h = rgb.rows;
    const int cur_w = rgb.cols;
    const double ratio = max(static_cast<double>(cur_w) / resize_w, static_cast<double>(cur_h) / resize_h);
    const int new_h = static_cast<int>(cur_h / ratio);
    const int new_w = static_cast<int>(cur_w / ratio);
    const int pad_h = max(0, resize_h - new_h);
    const int pad_w = max(0, resize_w - new_w);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    const size_t hw = static_cast<size_t>(resize_h) * resize_w;
    fill(out.begin() + base, out.begin() + base + 3 * hw, -1.0f);
    for (int y = 0; y < new_h; ++y) {
        const uint8_t* row = resized.ptr<uint8_t>(y);
        int oy = y + pad_h;
        for (int x = 0; x < new_w; ++x) {
            int ox = x + pad_w;
            size_t dst = static_cast<size_t>(oy) * resize_w + ox;
            out[base + dst] = row[x * 3 + 0] / 127.5f - 1.0f;
            out[base + hw + dst] = row[x * 3 + 1] / 127.5f - 1.0f;
            out[base + 2 * hw + dst] = row[x * 3 + 2] / 127.5f - 1.0f;
        }
    }
}

int run_robot(
    const Args& args,
    const RuntimeMeta& meta,
    SmolVLAPipeline& pipe,
    SmolVLAPipeline* cpu_shadow_pipe) {
    if (!args.no_motors && static_cast<int>(meta.calib.size()) != meta.state_dim) {
        cerr << "runtime metadata has no complete calibration; run export_smolvla_runtime.py with --calibration\n";
        return 2;
    }
    const int camera_count = pipe.camera_count();
    vector<string> camera_names;
    for (const auto& image_key : meta.image_keys) camera_names.push_back(image_key_to_camera_name(image_key));
    const int physical_camera_count = min(camera_count, static_cast<int>(camera_names.size()));
    if (camera_count > physical_camera_count) {
        cout << "[robot] model expects " << camera_count
            << " camera slots; using " << physical_camera_count
            << " configured camera(s) and filling remaining slot(s) with empty images\n";
    }

    vector<cv::VideoCapture> caps(physical_camera_count);
    const auto aliases = legacy_camera_aliases();
    for (int i = 0; i < physical_camera_count; ++i) {
        auto it = args.cam_index.find(camera_names.at(i));
        if (it == args.cam_index.end()) {
            auto alias_it = aliases.find(camera_names.at(i));
            if (alias_it != aliases.end()) it = args.cam_index.find(alias_it->second);
        }
        if (it == args.cam_index.end()) {
            cerr << "missing --camera " << camera_names.at(i) << "=IDX\n";
            return 2;
        }
        caps[i].open(it->second, cv::CAP_V4L2);
        if (!caps[i].isOpened()) {
            cerr << "cannot open camera " << camera_names.at(i) << " /dev/video" << it->second << "\n";
            return 2;
        }
        caps[i].set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        caps[i].set(cv::CAP_PROP_FRAME_WIDTH, 640);
        caps[i].set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        caps[i].set(cv::CAP_PROP_FPS, args.fps);
        cout << "[robot] camera " << camera_names.at(i) << " <- /dev/video" << it->second << "\n";
    }

    vector<struct motor_dev*> motors(meta.state_dim, nullptr);
    if (!args.no_motors) {
        for (int i = 0; i < meta.state_dim; ++i) {
            motors[i] = motor_alloc_uart(
                "drv_uart_feetech", args.port.c_str(), args.baud,
                static_cast<uint8_t>(meta.calib[i].id), nullptr);
            if (!motors[i]) {
                cerr << "motor_alloc failed: " << meta.motor_order.at(i) << "\n";
                return 2;
            }
        }
        if (motor_init(motors.data(), meta.state_dim) != 0) {
            cerr << "motor_init failed\n";
            return 2;
        }
        cout << "[robot] " << meta.state_dim << " motors on " << args.port
            << (args.dry_run ? " (DRY-RUN)" : "") << "\n";
    } else {
        cout << "[robot] motors disabled; using zero normalized state"
            << (args.dry_run ? " (DRY-RUN)" : "") << "\n";
    }

    const int n_steps = min(args.n_action_steps > 0 ? args.n_action_steps : meta.n_action_steps, meta.chunk_size);
    cout << "[robot] action queue n_action_steps=" << n_steps << " chunk_size=" << meta.chunk_size << "\n";
    const size_t image_elems = static_cast<size_t>(camera_count) * 3 * meta.resize_h * meta.resize_w;
    vector<float> images(image_elems);
    vector<float> state_norm_padded(meta.max_state_dim, 0.0f);
    vector<vector<float>> queue;
    size_t q_head = 0;

    const int loop_limit = args.max_iters > 0 ? args.max_iters : static_cast<int>(args.fps * args.episode_time);
    const double period = args.fps > 0 ? 1.0 / args.fps : 0.0;

    for (int iter = 0; iter < loop_limit && !g_stop; ++iter) {
        auto loop_start = steady_clock::now();

        fill(state_norm_padded.begin(), state_norm_padded.end(), 0.0f);
        if (!args.no_motors) {
            vector<struct motor_state> ms(meta.state_dim);
            motor_get_states(motors.data(), ms.data(), meta.state_dim);
            for (int i = 0; i < meta.state_dim; ++i) {
                int raw = static_cast<int>(llround(static_cast<double>(ms[i].pos) / kTwoPi * kMaxStep));
                raw = min(4095, max(0, raw));
                float state = static_cast<float>(raw_to_norm(meta.calib[i], raw));
                state_norm_padded[i] = (state - meta.state_mean[i]) / (meta.state_std[i] + 1e-8f);
            }
        }

        if (args.infer_every_tick || q_head >= queue.size()) {
            fill(images.begin(), images.end(), -1.0f);
            for (int ci = 0; ci < physical_camera_count; ++ci) {
                cv::Mat frame;
                caps[ci] >> frame;
                if (frame.empty()) {
                    cerr << "empty frame from camera " << camera_names.at(ci) << "\n";
                    continue;
                }
                preprocess_frame(
                    frame,
                    images,
                    static_cast<size_t>(ci) * 3 * meta.resize_h * meta.resize_w,
                    meta.resize_w,
                    meta.resize_h);
            }
            auto result = pipe.run(images, state_norm_padded);
            bool used_shadow = false;
            float shadow_diff = 0.0f;
            if (cpu_shadow_pipe != nullptr) {
                auto cpu_result = cpu_shadow_pipe->run(images, state_norm_padded);
                shadow_diff = max_abs_diff(result.actions, cpu_result.actions);
                if (!actions_finite(result.actions) || shadow_diff > args.shadow_action_tol) {
                    result = move(cpu_result);
                    used_shadow = true;
                }
            }
            queue.clear();
            q_head = 0;
            for (int step = 0; step < n_steps; ++step) {
                vector<float> action(meta.action_dim);
                copy(
                    result.actions.begin() + static_cast<size_t>(step) * meta.action_dim,
                    result.actions.begin() + static_cast<size_t>(step + 1) * meta.action_dim,
                    action.begin());
                queue.push_back(move(action));
            }
            cout << "[iter " << setw(3) << iter << "] infer="
                << fixed << setprecision(1)
                << (result.vision_ms + result.connector_ms + result.prefill_ms + result.denoise_ms)
                << " ms  v=" << result.vision_ms
                << " c=" << result.connector_ms
                << " pf=" << result.prefill_ms
                << " dn=" << result.denoise_ms;
            if (cpu_shadow_pipe != nullptr) {
                cout << " shadow_diff=" << shadow_diff
                    << (used_shadow ? " corrected=CPU" : " corrected=no");
            }
            cout << "\n";
        }

        const vector<float>& action = queue[q_head++];
        bool valid_action = true;
        for (int i = 0; i < meta.state_dim; ++i) {
            if (!isfinite(action[i])) {
                cerr << "non-finite action at joint " << i << "\n";
                valid_action = false;
                break;
            }
        }
        if (!valid_action) {
            g_stop = 1;
            continue;
        }
        if (!args.dry_run && !args.no_motors) {
            vector<struct motor_cmd> cmds(meta.state_dim);
            for (int i = 0; i < meta.state_dim; ++i) {
                int raw = norm_to_raw(meta.calib[i], action[i], !args.no_clamp);
                cmds[i].mode = MOTOR_MODE_POS;
                cmds[i].pos_des = static_cast<float>(static_cast<double>(raw) / kMaxStep * kTwoPi);
                cmds[i].vel_des = args.motor_speed;
                cmds[i].trq_des = 0.0f;
                cmds[i].kp = 0.0f;
                cmds[i].kd = 0.0f;
            }
            motor_set_cmds(motors.data(), cmds.data(), meta.state_dim);
        }
        if (args.print_actions) {
            cout << "  action=[";
            for (int i = 0; i < meta.action_dim; ++i)
                cout << fixed << setprecision(3) << action[i] << (i + 1 < meta.action_dim ? "," : "");
            cout << "]\n";
        }

        double dt = duration_cast<duration<double>>(steady_clock::now() - loop_start).count();
        if (period > dt) sleep_for(duration<double>(period - dt));
    }

    if (!args.no_motors) {
        vector<struct motor_cmd> idle(meta.state_dim);
        for (auto& c : idle) c.mode = MOTOR_MODE_IDLE;
        motor_set_cmds(motors.data(), idle.data(), meta.state_dim);
        motor_free(motors.data(), meta.state_dim);
    }
    for (auto& cap : caps) if (cap.isOpened()) cap.release();
    cout << "[robot] done\n";
    return 0;
}
#endif

}  // namespace

int main(int argc, char* argv[]) {
    try {
        install_signal_handlers();
        Args args = parse_args(argc, argv);
        if (args.use_spacemit_ep) set_spacemit_ep_env_defaults();
        RuntimeMeta meta = parse_runtime_meta(args.runtime);
        cout << "[smolvla_evaluate] model_dir=" << args.model_dir
            << " runtime=" << args.runtime
            << " task=\"" << meta.task << "\"\n";
        SmolVLAPipeline pipe(args, meta);
        unique_ptr<SmolVLAPipeline> cpu_shadow_pipe;
        if (args.cpu_shadow_correct) {
            Args cpu_args = args;
            cpu_args.use_spacemit_ep = false;
            cpu_args.vision_ep = false;
            cpu_args.connector_ep = false;
            cpu_args.prefill_ep = false;
            cpu_args.denoise_ep = false;
            cout << "[smolvla_evaluate] CPU shadow correction enabled, tol="
                << args.shadow_action_tol << "\n";
            cpu_shadow_pipe = make_unique<SmolVLAPipeline>(cpu_args, meta);
        }
        for (int i = 0; i < args.warmup; ++i) {
            vector<float> images(static_cast<size_t>(pipe.camera_count()) * 3 * meta.resize_h * meta.resize_w, -1.0f);
            vector<float> state(meta.max_state_dim, 0.0f);
            auto warmup_result = pipe.run(images, state);
            const double infer_ms = warmup_result.vision_ms
                + warmup_result.connector_ms
                + warmup_result.prefill_ms
                + warmup_result.denoise_ms;
            cout << "[warmup " << setw(3) << i << "] infer="
                << fixed << setprecision(1)
                << infer_ms
                << " ms  v=" << warmup_result.vision_ms
                << " c=" << warmup_result.connector_ms
                << " pf=" << warmup_result.prefill_ms
                << " dn=" << warmup_result.denoise_ms
                << "\n";
            if (args.print_actions) {
                cout << "[warmup " << i << "] action=[";
                for (int j = 0; j < meta.action_dim; ++j) {
                    cout << fixed << setprecision(6)
                        << warmup_result.actions[static_cast<size_t>(j)]
                        << (j + 1 < meta.action_dim ? "," : "");
                }
                cout << "]\n";
            }
            if (cpu_shadow_pipe) (void)cpu_shadow_pipe->run(images, state);
        }
        if (args.warmup_only) return 0;
#ifdef ACT_ROBOT_HW
        return run_robot(args, meta, pipe, cpu_shadow_pipe.get());
#else
        cerr << "smolvla_evaluate real-robot mode requires cmake -DACT_ROBOT_HW=ON\n";
        return 2;
#endif
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << "\nUse --help for usage.\n";
        return 1;
    }
}
