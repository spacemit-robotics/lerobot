/**
 * @file act_benchmark.cpp
 * @brief Benchmark ACT whole-graph ONNX inference on CPU or SpaceMIT EP.
 *
 * The benchmark uses deterministic inputs matching compare_act_onnx.py, can
 * optionally load .npy inputs / references, and reports latency statistics.
 */

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <cmath>
#include <random>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#include "onnxruntime_cxx_api.h"
#include "spacemit_ort_env.h"
#include "utils/common.h"
#include "utils/act_runtime.h"

using namespace std;
using namespace std::chrono;

struct Config {
    string model_path;
    bool   use_ep = false;
    int    threads = 4;
    string affinity;
    int    iters = 20;
    int    warmup = 3;
    unsigned seed = 0;
    int    cams = 2;
    int    height = 480;
    int    width = 640;
    int    state_dim = 6;
    string images_npy;   // optional: load images input from npy
    string state_npy;    // optional: load state input from npy
    string ref_npy;      // optional: reference actions for diff check
};

static void PrintUsage(const char* prog) {
    cout << "Usage: " << prog << " <model.onnx> [options]\n"
        << "  -s, --spacemit        Enable SpaceMIT EP (default CPU)\n"
        << "  -t, --threads N       Intra/EP thread count (default 4)\n"
        << "  -a, --affinity \"8;9\"  SpaceMIT EP core affinity\n"
        << "  -n, --iters N         Benchmark iterations (default 20)\n"
        << "  -w, --warmup N        Warmup iterations (default 3)\n"
        << "      --seed N          Input RNG seed (default 0)\n"
        << "      --cams N          Camera count (default 2)\n"
        << "      --height H        Image height (default 480)\n"
        << "      --width W         Image width (default 640)\n"
        << "      --state-dim N     State dim, 0 disables (default 6)\n"
        << "      --images-npy P    Load images input from .npy\n"
        << "      --state-npy P     Load state input from .npy\n"
        << "      --ref-npy P       Reference actions .npy for diff check\n";
}

static bool ParseArgs(int argc, char** argv, Config& c) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return false;
    }
    c.model_path = argv[1];
    for (int i = 2; i < argc; i++) {
        string a = argv[i];
        auto next = [&](const char* name) -> string {
            if (i + 1 >= argc) {
                cerr << name << " needs a value\n";
                exit(2);
            }
            return argv[++i];
        };
        if (a == "-s" || a == "--spacemit") c.use_ep = true;
        else if (a == "-t" || a == "--threads") c.threads = stoi(next("--threads"));
        else if (a == "-a" || a == "--affinity") c.affinity = next("--affinity");
        else if (a == "-n" || a == "--iters") c.iters = stoi(next("--iters"));
        else if (a == "-w" || a == "--warmup") c.warmup = stoi(next("--warmup"));
        else if (a == "--seed") c.seed = (unsigned)stoul(next("--seed"));
        else if (a == "--cams") c.cams = stoi(next("--cams"));
        else if (a == "--height") c.height = stoi(next("--height"));
        else if (a == "--width") c.width = stoi(next("--width"));
        else if (a == "--state-dim") c.state_dim = stoi(next("--state-dim"));
        else if (a == "--images-npy") c.images_npy = next("--images-npy");
        else if (a == "--state-npy") c.state_npy = next("--state-npy");
        else if (a == "--ref-npy") c.ref_npy = next("--ref-npy");
        else if (a == "-h" || a == "--help") {
            PrintUsage(argv[0]);
            exit(0);
        } else {
            cerr << "unknown arg: " << a << "\n";
            PrintUsage(argv[0]);
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    Config cfg;
    if (!ParseArgs(argc, argv, cfg)) return 1;

    cout << "[act] model=" << cfg.model_path
        << " provider=" << (cfg.use_ep ? "SpaceMIT EP" : "CPU")
        << " threads=" << cfg.threads;
    if (cfg.use_ep && !cfg.affinity.empty()) cout << " affinity=" << cfg.affinity;
    cout << endl;

    ActOrtConfig ort_cfg{cfg.model_path, cfg.use_ep, cfg.threads, cfg.affinity};
    ActOnnxRunner runner(ort_cfg);
    Ort::AllocatorWithDefaultOptions alloc;

    // Resolve input/output names and order from the graph.
    vector<string> in_names, out_names;
    for (const auto& name : runner.in_names) in_names.push_back(name);
    for (const auto& name : runner.out_names) out_names.push_back(name);
    vector<const char*> in_ptrs, out_ptrs;
    for (auto& s : in_names) in_ptrs.push_back(s.c_str());
    for (auto& s : out_names) out_ptrs.push_back(s.c_str());

    // ---- Build inputs ------------------------------------------------------
    mt19937 gen(cfg.seed);
    normal_distribution<float> normal_dist(0.0f, 1.0f);

    vector<float> images, state;
    vector<int64_t> img_shape, st_shape;

    if (!cfg.images_npy.empty()) {
        images = LoadNpyF32(cfg.images_npy, img_shape);
        if (img_shape.size() == 4) {
            // Accept [cams,3,H,W] by adding a batch dim.
            img_shape.insert(img_shape.begin(), 1);
        }
        if (img_shape.size() != 5) {
            throw runtime_error("images.npy must be 4D or 5D: [cams,3,H,W] or [1,cams,3,H,W]");
        }
        if (img_shape[1] != cfg.cams || img_shape[2] != 3 ||
            img_shape[3] != cfg.height || img_shape[4] != cfg.width) {
            throw runtime_error(
                string("images.npy shape mismatch: expected [1,") +
                to_string(cfg.cams) + ",3," + to_string(cfg.height) + "," +
                to_string(cfg.width) + "]");
        }
    } else {
        img_shape = {1, cfg.cams, 3, cfg.height, cfg.width};
        size_t n = 1;
        for (auto d : img_shape) n *= d;
        images.resize(n);
        for (auto& v : images) v = normal_dist(gen);
    }
    if (cfg.state_dim > 0) {
        if (!cfg.state_npy.empty()) {
            state = LoadNpyF32(cfg.state_npy, st_shape);
            if (st_shape.size() == 1) {
                st_shape.insert(st_shape.begin(), 1);
            }
            if (st_shape.size() != 2) {
                throw runtime_error("state.npy must be 1D or 2D: [state_dim] or [1,state_dim]");
            }
            if (st_shape[0] != 1 || st_shape[1] != cfg.state_dim) {
                throw runtime_error(
                    string("state.npy shape mismatch: expected [1,") +
                    to_string(cfg.state_dim) + "]");
            }
        } else {
            st_shape = {1, cfg.state_dim};
            state.resize((size_t)cfg.state_dim);
            for (auto& v : state) v = normal_dist(gen);
        }
    }

    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto make_feed = [&]() {
        vector<Ort::Value> vals;
        for (auto& name : in_names) {
            if (name == "images")
                vals.push_back(Ort::Value::CreateTensor<float>(
                    mem, images.data(), images.size(), img_shape.data(), img_shape.size()));
            else if (name == "state")
                vals.push_back(Ort::Value::CreateTensor<float>(
                    mem, state.data(), state.size(), st_shape.data(), st_shape.size()));
            else
                throw runtime_error("unhandled input: " + name);
        }
        return vals;
    };

    // ---- Warmup ------------------------------------------------------------
    vector<Ort::Value> last_out;
    for (int i = 0; i < cfg.warmup; i++) {
        auto feed = make_feed();
        last_out = runner.session->Run(Ort::RunOptions{nullptr}, in_ptrs.data(),
            feed.data(), feed.size(), out_ptrs.data(), out_ptrs.size());
    }

    // ---- Benchmark ---------------------------------------------------------
    Timer t;
    vector<double> times;
    times.reserve(cfg.iters);
    for (int i = 0; i < cfg.iters; i++) {
        auto feed = make_feed();
        t.Start();
        last_out = runner.session->Run(Ort::RunOptions{nullptr}, in_ptrs.data(),
            feed.data(), feed.size(), out_ptrs.data(), out_ptrs.size());
        times.push_back(t.ElapsedMs());
    }

    double mean = 0, mn = times[0], mx = times[0];
    for (double v : times) { mean += v; mn = min(mn, v); mx = max(mx, v); }
    mean /= times.size();
    sort(times.begin(), times.end());
    double median = times[times.size() / 2];

    auto info = last_out[0].GetTensorTypeAndShapeInfo();
    auto oshape = info.GetShape();
    cout << "[act] output shape=[";
    for (size_t i = 0; i < oshape.size(); i++) cout << (i ? "," : "") << oshape[i];
    cout << "]" << endl;
    cout << "[act] latency mean=" << mean << "ms median=" << median
        << "ms min=" << mn << "ms max=" << mx
        << " (iters=" << cfg.iters << ", warmup=" << cfg.warmup << ")" << endl;

    // ---- Optional reference diff ------------------------------------------
    if (!cfg.ref_npy.empty()) {
        vector<int64_t> ref_shape;
        vector<float> ref = LoadNpyF32(cfg.ref_npy, ref_shape);
        const float* out = last_out[0].GetTensorData<float>();
        size_t n = info.GetElementCount();
        if (ref.size() != n) {
            cerr << "[act] ref size " << ref.size() << " != output size " << n << endl;
        } else {
            double max_abs = 0, max_rel = 0;
            for (size_t i = 0; i < n; i++) {
                double d = fabs((double)out[i] - (double)ref[i]);
                max_abs = max(max_abs, d);
                double den = fabs((double)ref[i]) + 1e-8;
                max_rel = max(max_rel, d / den);
            }
            cout << "[act] diff vs ref: max|abs|=" << max_abs
                << " max|rel|=" << max_rel
                << (max_abs < 1e-3 ? "  OK" : "  CHECK") << endl;
        }
    }
    return 0;
}
