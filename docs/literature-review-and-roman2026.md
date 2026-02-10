# 📚 Literature Review & Related Work

## Has Anyone Done This Before?

**Short answer: Pieces exist, but nobody has done exactly what we're proposing.** That's good for the thesis — there's a clear gap.

---

## ✅ What EXISTS (Related Work)

### 1. VLA (Vision-Language-Action) End-to-End Models
These go even further than our approach — single model does vision + language + action:

| Paper | Year | What It Does | Why It's Different From Ours |
|-------|:----:|-------------|-------------------------------|
| **RT-2** (Google DeepMind) | 2023 | Single VLA model: image + language → robot action tokens directly | Massive model (55B), cloud-only, not edge, not distributed |
| **VIMA** | 2022 | Multi-modal imitation learning for manipulation | Simulation-focused, not edge deployed |
| **SayCan** (Google) | 2022 | LLM plans → robot skill library executes | Uses pre-defined skill primitives, not VLM grounding for localization |
| **SmolVLA** | 2025 | Compact VLA (<1B params) for real-world manipulation | End-to-end action model, not LLM+VLM pipeline like ours |

### 2. VLM Grounding for Robotics (No YOLO)

| Paper | Year | What It Does |
|-------|:----:|-------------|
| **Florence-VL** ([arXiv:2412.04424](https://arxiv.org/abs/2412.04424)) | Dec 2024 | Florence-2 + LLM fusion for grounding/reasoning — shows VLM can replace detectors |
| **Qwen2.5-VL** ([arXiv:2502.13923](https://arxiv.org/abs/2502.13923)) | Feb 2025 | Native bbox grounding from text prompts — directly applicable to our pipeline |

### 3. Edge LLM/VLM Benchmarks on Jetson

| Paper | Year | What It Does |
|-------|:----:|-------------|
| **NanoVLA** (OpenReview 2025) | 2025 | Nano-scale VLA on Jetson Orin Nano — 52x faster than SOTA, edge-optimized |
| **LLM Inferencing on Edge** ([arXiv:2506.09554](https://arxiv.org/abs/2506.09554)) | 2025 | Benchmarks Llama3, Phi2, DeepSeek on Jetson Orin AGX — latency, power, throughput |
| **EdgeReasoning** ([arXiv:2511.01866](https://arxiv.org/abs/2511.01866)) | 2025 | Latency-accuracy tradeoffs for reasoning LLMs on edge GPUs |
| **NVIDIA Cosmos Nemotron** | 2024 | 4-bit quantized VLMs on Jetson Orin — near full-precision accuracy |
| **NVIDIA Jetson MLPerf Benchmarks** | 2024-25 | Official benchmarks for Qwen2.5-VL, Llama 3.2 Vision on Orin |

### 4. VLA Efficiency / Pruning

| Paper | Year | What It Does |
|-------|:----:|-------------|
| **SpecPrune-VLA** ([arXiv:2509.05614](https://arxiv.org/abs/2509.05614)) | 2025 | Token/layer pruning for VLA speedup — 1.7x faster, minimal accuracy drop |
| **ThinkProprio** ([arXiv:2602.06575](https://arxiv.org/abs/2602.06575)) | 2026 | Proprioceptive fusion into VLM — 50% latency reduction on Franka |

---

## ❌ What NOBODY Has Done (Our Gap)

| Gap | Why It Matters |
|-----|---------------|
| **Comparing multiple grounding VLMs (Florence-2, Qwen-VL, LLaVA, VILA) head-to-head for robotic pick & place** | Everyone evaluates their own model — no unified comparison across VLMs for manipulation |
| **LLM + VLM distributed pipeline on Jetson for real Franka pick & place** | RT-2/SayCan use cloud, NanoVLA is end-to-end (not LLM+VLM split), nobody does distributed edge |
| **Comparing LLM planners (Llama3, Mistral, Phi-3) paired with different VLMs** | LLM benchmarks exist for text tasks, but not for robotic task planning quality |
| **Model comparison specifically for bbox grounding accuracy → depth → pick & place success** | Grounding benchmarks exist on RefCOCO etc., but NOT connected to real robot success rate |
| **Edge-only, no cloud, no YOLO, pure LLM+VLM manipulation** | Most work either uses cloud or includes a traditional detector somewhere |

---

## 🎯 Thesis Novelty

Our work sits in an **uncovered intersection**:

```
                    Edge Deployment (Jetson)
                           ∩
              LLM+VLM Distributed Pipeline
                           ∩
           Multiple VLM Model Comparison
                           ∩
         Real Robot Pick & Place (Franka FR3)
                           ∩
              No Traditional Object Detector
```

**Nobody has published this exact combination.** The closest works are:
- **NanoVLA** — edge VLA but end-to-end, not comparing models
- **RT-2** — VLM+action but cloud, single model, not comparing
- **LLM Edge benchmarks** — measure latency but not connected to manipulation success

---

## 📖 Key Papers to Cite

| Paper | Cite As | Why |
|-------|---------|-----|
| RT-2 ([arXiv:2307.15818](https://arxiv.org/abs/2307.15818)) | VLA baseline, cloud approach | Shows VLM can drive manipulation, but cloud-only |
| SayCan ([arXiv:2204.01691](https://arxiv.org/abs/2204.01691)) | LLM planning for robots | Established LLM → skill execution paradigm |
| Florence-VL ([arXiv:2412.04424](https://arxiv.org/abs/2412.04424)) | VLM grounding capability | Shows Florence can replace object detectors |
| Qwen2.5-VL ([arXiv:2502.13923](https://arxiv.org/abs/2502.13923)) | VLM with native bbox | Direct competitor for our grounding VLM |
| NanoVLA (OpenReview 2025) | Edge VLA baseline | Shows edge manipulation is feasible at nano scale |
| LLM on Edge ([arXiv:2506.09554](https://arxiv.org/abs/2506.09554)) | Edge LLM benchmarks | Latency/power reference for Jetson Orin |
| EdgeReasoning ([arXiv:2511.01866](https://arxiv.org/abs/2511.01866)) | Edge reasoning tradeoffs | Latency-accuracy analysis framework |
| NVIDIA Jetson Benchmarks | Hardware reference | Official throughput numbers for our platform |

---

---

# 🏛️ IEEE RO-MAN 2026 — Conference Info

## Key Dates

| Milestone | Date |
|-----------|------|
| **Paper Submission Deadline** | **March 15, 2026** |
| Notification of Acceptance | May 29, 2026 |
| Camera-Ready Submission | June 20, 2026 |
| Late Breaking Reports Submission | June 12, 2026 |
| Late Breaking Reports Notification | June 26, 2026 |
| **Conference** | **August 24–28, 2026** |

## Location

📍 **Kitakyushu, Fukuoka, Japan**

## Acceptance Rate

- The exact acceptance rate for RO-MAN 2026 won't be known until after notifications (late May 2026).
- **Historical RO-MAN acceptance rates** have typically been in the **~40–50%** range, making it a moderately competitive venue.
- Official site: [ro-man2026.org](https://ro-man2026.org/)

## Will Our Paper Get Accepted?

There's no guaranteed answer, but here's an honest assessment:

### ✅ Strengths (Why It Could Get Accepted)
- **Clear novelty gap** — nobody has published the exact LLM+VLM distributed edge pipeline for real robot manipulation with multi-model comparison
- **Real hardware** — Franka FR3 + Jetson Orin is a compelling real-world setup, not just simulation
- **Practical relevance** — edge deployment without cloud is highly relevant to the robotics community
- **Timely topic** — VLMs and LLMs for robotics are hot right now; RO-MAN reviewers will find it interesting
- **Systematic comparison** — comparing multiple VLMs and LLMs adds scientific rigor beyond a single-system demo

### ⚠️ Risks (What Could Hurt)
- **Depth vs. breadth** — comparing many models might feel shallow if each isn't analyzed deeply; make sure to include failure analysis
- **RO-MAN focus** — RO-MAN emphasizes *human-robot interaction*; make sure to frame the work around how natural language commands from humans drive the system (the "human interactive communication" angle)
- **Baselines** — reviewers will want to see comparison against at least one traditional pipeline (e.g., YOLO-based) to justify the VLM-only approach
- **Reproducibility** — open-source the code and provide clear setup instructions (this repo helps!)

### 💡 Recommendation
- **Frame it as HRI**: "Humans give natural language commands → LLM plans → VLM grounds → robot acts" — this is a human-robot interaction story
- **Include a user study** if possible (even small) — RO-MAN loves user studies
- **Deadline is March 15, 2026** — you have ~33 days from today (Feb 10, 2026)

> **Bottom line:** With solid experiments, good framing for the HRI audience, and the clear novelty gap identified above, this has a **good chance** at RO-MAN 2026. The acceptance rate (~40–50%) is in your favor compared to top-tier venues like ICRA/RSS.