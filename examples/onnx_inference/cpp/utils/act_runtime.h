/**
 * @file act_runtime.h
 * @brief Minimal ONNX Runtime wrapper for ACT whole-graph inference.
 */

#ifndef ONNX_INFERENCE_ACT_CPP_UTILS_ACT_RUNTIME_H_
#define ONNX_INFERENCE_ACT_CPP_UTILS_ACT_RUNTIME_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "onnxruntime_cxx_api.h"
#include "spacemit_ort_env.h"

struct ActOrtConfig {
    std::string model_path;
    bool use_ep = false;
    int threads = 4;
    std::string affinity;
};

struct ActOnnxRunner {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "act_runtime"};
    Ort::SessionOptions opts;
    std::unique_ptr<Ort::Session> session;
    std::vector<std::string> in_names, out_names;

    explicit ActOnnxRunner(const ActOrtConfig& cfg) {
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        opts.SetIntraOpNumThreads(cfg.threads);
        if (cfg.use_ep) {
            std::unordered_map<std::string, std::string> popts;
            popts["SPACEMIT_EP_INTRA_THREAD_NUM"] = std::to_string(cfg.threads);
            if (!cfg.affinity.empty())
                popts["SPACEMIT_EP_INTRA_THREAD_AFFINITY"] = cfg.affinity;
            Ort::SessionOptionsSpaceMITEnvInit(opts, popts);
            opts.EnableMemPattern();
            opts.EnableCpuMemArena();
        }
        session = std::make_unique<Ort::Session>(env, cfg.model_path.c_str(), opts);

        Ort::AllocatorWithDefaultOptions alloc;
        for (size_t i = 0; i < session->GetInputCount(); ++i)
            in_names.push_back(session->GetInputNameAllocated(i, alloc).get());
        for (size_t i = 0; i < session->GetOutputCount(); ++i)
            out_names.push_back(session->GetOutputNameAllocated(i, alloc).get());
    }

    /**
     * @brief Run one ACT forward pass and return the normalized action chunk.
     *
     * @param images Normalized image tensor storage.
     * @param img_shape Shape for the images input, usually [1, n_cam, 3, H, W].
     * @param state Normalized robot state tensor storage.
     * @param st_shape Shape for the state input, usually [1, state_dim].
     * @param chunk Output action chunk length.
     * @param adim Output action dimension.
     * @return Flattened [chunk, adim] action values in normalized space.
     */
    std::vector<float> RunChunk(
        std::vector<float>& images,
        const std::vector<int64_t>& img_shape,
        std::vector<float>& state,
        const std::vector<int64_t>& st_shape,
        int& chunk,
        int& adim) {
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<Ort::Value> ins;
        std::vector<const char*> in_ptr, out_ptr;
        for (auto& n : in_names) {
            if (n == "images") {
                ins.push_back(Ort::Value::CreateTensor<float>(
                    mem, images.data(), images.size(), img_shape.data(), img_shape.size()));
            } else if (n == "state") {
                ins.push_back(Ort::Value::CreateTensor<float>(
                    mem, state.data(), state.size(), st_shape.data(), st_shape.size()));
            }
        }
        for (auto& s : in_names) in_ptr.push_back(s.c_str());
        for (auto& s : out_names) out_ptr.push_back(s.c_str());
        auto outs = session->Run(Ort::RunOptions{nullptr}, in_ptr.data(), ins.data(),
                                 ins.size(), out_ptr.data(), out_ptr.size());
        auto shp = outs[0].GetTensorTypeAndShapeInfo().GetShape();
        chunk = static_cast<int>(shp[1]);
        adim = static_cast<int>(shp[2]);
        const float* p = outs[0].GetTensorData<float>();
        return std::vector<float>(p, p + static_cast<size_t>(chunk) * adim);
    }
};

#endif  // ONNX_INFERENCE_ACT_CPP_UTILS_ACT_RUNTIME_H_
