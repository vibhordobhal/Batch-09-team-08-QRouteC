# QRoute-C: A Memory-Constrained Q-Learning Framework for Autonomous Robot Path Planning

## 📋 Team & Project Details
* **Project Title:** QRoute-C: A Memory-Constrained Q-Learning Framework for Autonomous Robot Path Planning
* **Institution:** University of Petroleum and Energy Studies (UPES)
* **Faculty Mentor:** Subhranil Das
* **Batch:** Batch 09
* **Team Number:** Team 08
* **Team Members:**
  1. **Ojasvi Chaurasia** – `590033520`
  2. **Varun Bhandari** – `590039446`
  3. **Abhinav Upadhyay** – `590036576`
  4. **Vibhor Dobhal** – `590032770`
* **GitHub Repository Name:** `Batch09_Team08_QRouteC`

---

## 🎯 Problem Statement
Autonomous mobile robots and embedded microcontrollers often operate under severe hardware constraints—specifically limited static RAM (SRAM), no Floating-Point Unit (FPU), and zero tolerance for dynamic heap fragmentation (`malloc`/`free`). Standard Reinforcement Learning (Q-Learning) implementations rely on 32-bit or 64-bit floating-point arrays, consuming excessive memory and processing cycles.

**Objective:** Design and implement a deterministic, memory-constrained Q-learning path-planning engine in C that enables a robot to autonomously navigate a 6×6 obstacle-filled grid world from Start (`S`) to Goal (`G`) while locking Q-table memory usage to strictly **288 bytes**.

---

## ✨ Objectives & Key Features
* **Strict Static Memory Budget:** Stores the entire $36 \times 4$ Q-table in **288 bytes** using scaled `int16_t` fixed-point arithmetic (total static data footprint: **432 bytes**; dynamic heap usage: **0 bytes**).
* **32-Bit Overflow-Safe Bellman Math:** Prevents signed 16-bit integer wrap-around (`> 32,767`) by casting intermediate products to `int32_t` before scaling down and clamping back to `int16_t`.
* **Autonomous Q-Learning Training Loop:** Trains over 1,000 episodes using an $\epsilon$-greedy exploration policy ($\epsilon = 0.20$, $\alpha = 0.15$, $\gamma = 0.90$) and fixed-point reward penalties (`+10000` Goal, `-1000` Obstacle/Boundary, `-10` Step).
* **Interactive CLI & Policy Visualization:** Features a 5-option terminal menu to view the map, train the agent, trace the learned collision-free path (`*`), and audit real-time byte usage.

---

## 🧠 Algorithm & Flowchart

### Pseudocode (QRoute-C Fixed-Point Learning Cycle)
```text
INITIALIZE q_table[36][4] = 0
FOR episode = 1 TO 1000:
    SET robot position (x, y) = (0, 0)
    FOR step = 1 TO 100:
        state = y * 6 + x
        IF rand() % 100 < 20:
            action = random_action(0..3)          // Exploration
        ELSE:
            action = argmax_a(q_table[state][a])  // Exploitation

        reward = move_robot(&x, &y, action)
        next_state = y * 6 + x
        max_next_q = max_a(q_table[next_state][a])

        // 32-bit overflow-safe fixed-point Bellman update
        target = reward + (90 * (int32_t)max_next_q) / 100
        delta  = (15 * (target - q_table[state][action])) / 100
        q_table[state][action] = clamp_int16(q_table[state][action] + delta)

        IF reward == GOAL_REWARD OR reward == OBSTACLE_REWARD OR reward == BOUNDARY_REWARD:
            BREAK
```

### System Flow Diagram
```text
[ Start Program ]
       │
       ▼
[ Initialize Static 6x6 Grid & 288-Byte Q-Table ]
       │
       ▼
┌──────────────────────────────────────────────────────────┐
│                 Interactive Menu (1 - 5)                 │
├──────────────┬───────────────┬──────────────┬────────────┤
│ 1. Show Map  │ 2. Train RL   │ 3. Show Path │ 4. Memory  │
│ (S, #, ., G) │ (1000 Eps)    │ (Traces *)   │ (432 B)    │
└──────────────┴───────┬───────┴──────────────┴────────────┘
                       │
                       ▼
        [ Option 5: Exit Simulation ]
```

---

## 💻 C Programming Concepts Used
* **Fixed-Width Integer Types (`<stdint.h>`):** `int16_t` for compact Q-table storage and `int32_t` for overflow-safe intermediate multiplication.
* **Structures & Pointers:** `QAgent` struct passed by reference (`QAgent *agent`) to functions for zero-copy state updates and coordinate modification (`int *x, int *y`).
* **2D & 1D Array Mapping:** 2D `grid[6][6]` environment representation mapped to 1D state indices (`y * GRID_SIZE + x`).
* **Control Flow & Modular Functions:** `do-while` interactive menu loop, `switch-case` dispatcher, nested `for` training loops, and modular helper functions.
* **Memory Inspection (`sizeof` & `memset`):** Compile-time byte verification (`sizeof`) and zero-initialization (`memset`) without `<stdlib.h>` heap functions.

---

## ⚙️ Compilation & Execution Instructions

### Using GCC (Linux / macOS / VS Code Terminal)
```bash
# 1. Compile the source code
gcc qroute.c -o qroute

# 2. Run the executable
./qroute
```

### Using Windows (Command Prompt / PowerShell / MinGW)
```cmd
gcc qroute.c -o qroute.exe
qroute.exe
```

---

## 🧪 Sample Test Cases (Expected vs. Actual Outputs)

### Test Case 1: Display Environment Map (Menu Option `1`)
* **Input:** `1`
* **Expected Output:** Displays the 6×6 grid with Start `S` at `(0,0)`, Goal `G` at `(5,5)`, and Obstacles `#`.
* **Actual Output:**
```text
--- QRoute-C Environment Map (6x6) ---
 S  .  .  .  .  . 
 .  #  #  .  #  . 
 .  .  .  .  #  . 
 .  #  #  .  .  . 
 .  .  #  #  #  . 
 .  .  .  .  .  G 
Legend: S = Start | G = Goal | # = Obstacle | . = Empty
```

### Test Case 2: Train Robot Agent (Menu Option `2`)
* **Input:** `2`
* **Expected Output:** Runs 1,000 Q-learning episodes, prints progress every 200 episodes, and completes without integer overflow.
* **Actual Output:**
```text
[TRAINING] Starting Q-Learning over 1000 episodes...
[PROGRESS] Episode 200 / 1000 completed.
[PROGRESS] Episode 400 / 1000 completed.
[PROGRESS] Episode 600 / 1000 completed.
[PROGRESS] Episode 800 / 1000 completed.
[PROGRESS] Episode 1000 / 1000 completed.
[SUCCESS] Training completed successfully using fixed-point int16_t math!
```

### Test Case 3: Display Autonomous Learned Path (Menu Option `3` after Training)
* **Input:** `3`
* **Expected Output:** Traces a valid, collision-free path marked with `*` from `S` to `G`.
* **Actual Output:**
```text
--- Autonomous Learned Path ---
 S  *  *  *  *  * 
 .  #  #  *  #  * 
 .  .  .  *  #  * 
 .  #  #  *  *  * 
 .  .  #  #  #  * 
 .  .  .  .  .  G 
Legend: S = Start | G = Goal | # = Obstacle | * = Learned Path
```

### Test Case 4: Memory Footprint Verification (Menu Option `4`)
* **Input:** `4`
* **Expected Output:** Confirms Q-table size is 288 bytes and total data memory is 432 bytes with zero dynamic allocation.
* **Actual Output:**
```text
--- Memory Footprint Analysis ---
Grid Memory: 144 bytes
Q-Table Memory (int16_t): 288 bytes
Total Data Memory: 432 bytes (Zero Dynamic Allocation)
States: 36 | Actions: 4 | Total Q-Values: 144
```

### Test Case 5: Clean Exit (Menu Option `5`)
* **Input:** `5`
* **Expected Output:** Terminates the loop cleanly.
* **Actual Output:**
```text
Exiting QRoute-C simulation
```

---

## 📁 Repository Structure
```text
Batch09_Team08_QRouteC/
├── README.md
├── qroute.c
├── presentation.pptx
└── screenshots/
    └── sample_output.png
```
