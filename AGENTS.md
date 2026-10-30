# Repository Guidelines

## Project Structure & Module Organization

The ROS 2 Humble/C++17 package mirrors headers in `include/vln_memory/bag/` with implementations in `src/bag/`: `common`, `rgb`, `depth`, `camera_info`, `tf`, and `builder`. Keep ROS adapters in `*_ros.hpp`/`*_ros.cpp`.

Tests mirror modules under `test/bag/`; `test/install_consumer/` validates installed interfaces. Categorize documentation under `docx/{project,common,modules,interfaces,reviews}/`. Ignore generated `build/`, `install/`, and `log/`.

## Scope & Development Workflow

Read `docx/project/sections_1_2_scope.md` and `docx/project/system_plan.md` before extending functionality. Scope: design sections 1–2; distinguish implementation from plans.

Plan inputs, outputs, ownership, dependencies, and errors; implement incrementally and review the full call path. Add same-stem module documentation and update `docx/README.md`.

## Build, Test, and Development Commands

Standalone core build and tests:

```bash
cmake -S . -B build/standalone -DVLN_MEMORY_ENABLE_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/standalone --parallel 2
ctest --test-dir build/standalone --output-on-failure
```

ROS adapter tests:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select vln_memory
colcon test-result --verbose
```

Run `ros2 run vln_memory bag_builder --config config/vln_memory.yaml` after editing sensor bindings.

## Coding Style & Naming Conventions

Follow `.editorconfig`: UTF-8, LF, two-space indentation. Use `snake_case` filenames, `PascalCase` types, `camelCase` functions, and namespace `vln_memory`.

- Avoid deeply nested data structures and nested `struct` definitions; prefer flat contracts.
- Do not use nested generator expressions or comprehensions; expand them into explicit `for` loops.
- Do not nest ternary expressions; use `if`/`else` branches.
- Keep functions short and focused; split long flows into coherent helpers.
- Name variables for their roles, meaning, and units; avoid vague abbreviations.
- Add function-level comments to every function, including helpers: explain purpose, inputs, outputs, and relevant ownership, units, and failures.

Preserve timestamps and provenance. No mandatory formatter/lint gate exists.

## Code Review Requirements

Analyze time and space complexity; optimize unnecessary scans, repeated computation, and copying without obscuring correctness. Check boundary conditions, overflow, lifetimes, error paths, and interface contracts. Add targeted regression tests for fixes.

## Testing Guidelines

CTest runs C++ check programs: `test_<module>.cpp`, with `_ros` for real-message adapters. Cover changed behavior, invalid inputs, ownership, and numerical boundaries. Validate exported headers/targets through installed consumers. Do not describe synthetic tests as real-bag validation. No coverage threshold is configured.

## Commit & Pull Request Guidelines

Use imperative commit subjects, following history: `Add TF coverage index and observation layer audit`. Use `codex/` branches, stage completed work, preserve unrelated edits, and leave merging for owner review. PRs should describe behavior, interfaces, validation, limitations, and relevant issues/documentation.

Keep credentials, recordings, and generated evidence out of Git.
