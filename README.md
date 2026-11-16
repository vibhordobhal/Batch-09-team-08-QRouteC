🤖 QRoute-C
A Memory-Constrained Q-Learning Framework for Autonomous Robot Path Planning

Teaching a robot to find its way, using only 288 bytes of learned memory.

<br>

Show Image Show Image Show Image Show Image Show Image

B.Tech CSE · UPES · Batch 09 / Team 08 · Faculty: Subhranil Das

Overview • Results • How it works • Run it • Dev log • Limitations

</div>
👥 Team
👤 Member
Ojasvi Chaurasia
Varun Bhandari
Abhinav Upadhyay
Vibhor Dobhal
📖 Overview

QRoute-C is a C program where a robot learns to travel from a start cell to a goal cell on a 6×6 obstacle grid using Q-learning, while keeping everything it learns in a tiny, fixed block of memory.

It is built for microcontrollers: integer-only fixed-point math, no dynamic allocation, and no FPU required.

💡 The core question: Can a robot learn useful paths while keeping its Q-value storage within a limited memory budget?

🧩 The problem
📈 Growing state space	🗄️ Q-table storage	🔌 Limited hardware
A bigger grid means more positions to learn.	A Q-table grows with the number of states.	Small robots have little RAM and often no FPU.
🛠️ The approach

The code was generated and refined through a seven-stage AI prompt-engineering workflow. Each stage's output was reviewed, bugs were found, and fixes were fed into the next prompt (see the development log).

🏆 Key Results
<div align="center">
Metric	🔴 Float baseline	🟢 QRoute-C final
Q-table type	float (4 B)	int16_t (2 B)
Q-table size	576 bytes	288 bytes
Q-table reduction	n/a	50%
Heap allocation	n/a	None
Floating-point math	Required	Not used
Total data (grid + Q-table)	720 bytes	432 bytes
</div>
text
36 states (6x6) x 4 actions x 4 bytes (float)   = 576 bytes
36 states (6x6) x 4 actions x 2 bytes (int16_t) = 288 bytes   <- 50% smaller
🧠 How It Works
Q-learning in four words
	Term	In this project
📍	State	Current cell on the 6×6 grid (36 states)
🎮	Action	Up, down, left or right (4 actions)
🎁	Reward	Goal bonus, collision penalty or step cost
⭐	Q-value	Score for how good an action is in a state
👁️ Observe state
🎯 Choose action
🎁 Receive reward
✏️ Update Q-value

The update rule:

text
Q(s,a) <- Q(s,a) + alpha * [ r + gamma * max Q(s',a') - Q(s,a) ]

alpha = learning rate · gamma = discount factor · r = reward · s' = next state

🔢 Fixed-point arithmetic

Every value is stored as an integer multiplied by SCALE_FACTOR = 100, so no floats are needed. A stored 3710 means 37.10.

<details> <summary><b>📐 Worked example of one update (click to expand)</b></summary> <br>

Using alpha = 0.20, gamma = 0.90 (the Prompt 3 values):

text
old Q = 0, reward = 100.00, best next Q = 95.00
target = 100 + 0.9 x 95     = 185.5
delta  = 0.2 x (185.5 - 0)  = 37.1
new Q  = 37.1    (stored as 3710)
</details>
🛡️ Overflow-safe update

Scaled products overflow int16_t easily (for example 90 × 9500 = 855,000, but the limit is 32,767). The final update:

Promotes operands to int32_t before multiplying
Does all intermediate math in 32-bit
Clamps the result to −32,768 … 32,767
Narrows to int16_t only for storage
<details> <summary><b>💻 See the final update function</b></summary> <br>
c
void update_q_value(QAgent *agent, int state, int action,
                    int reward, int next_state) {
    int16_t max_next_q = agent->q_table[next_state][0];
    for (int a = 1; a < NUM_ACTIONS; a++)
        if (agent->q_table[next_state][a] > max_next_q)
            max_next_q = agent->q_table[next_state][a];

    int32_t future_scaled = ((int32_t)GAMMA_SCALED * (int32_t)max_next_q) / SCALE_FACTOR;
    int32_t target     = (int32_t)reward + future_scaled;
    int32_t current_q  = (int32_t)agent->q_table[state][action];
    int32_t diff       = target - current_q;
    int32_t adjustment = (ALPHA_SCALED * diff) / SCALE_FACTOR;
    int32_t new_q      = current_q + adjustment;

    if (new_q >  32767) new_q =  32767;   // saturate high
    if (new_q < -32768) new_q = -32768;   // saturate low
    agent->q_table[state][action] = (int16_t)new_q;
}
</details>
🏗️ System Architecture

Four logical modules in a single C program:

feeds next decision
🗺️ 1. Grid Environmentgrid, start, goal, obstacles
🤖 2. Agentcurrent state + actions
🧠 3. Q-Learningchoose actions, updateQ-values
💾 4. Memory Managerstore and retrieve Q-values
📤 Outputslearned path, memoryusage

The memory manager is the statically allocated int16_t q_table[NUM_STATES][NUM_ACTIONS] inside the agent struct, and memory is reported at runtime with sizeof().

🗺️ Environment and Parameters
The grid

🤖 start (0,0) · 🏁 goal (5,5) · ⬛ obstacle · ⬜ free

text
🤖 ⬜ ⬜ ⬜ ⬜ ⬜
⬜ ⬛ ⬛ ⬜ ⬛ ⬜
⬜ ⬜ ⬜ ⬜ ⬛ ⬜
⬜ ⬛ ⬛ ⬜ ⬜ ⬜
⬜ ⬜ ⬛ ⬛ ⬛ ⬜
⬜ ⬜ ⬜ ⬜ ⬜ 🏁

Stored as int grid[6][6]: 0 empty, 1 obstacle, 2 start, 3 goal.

🎁 Rewards (scaled by 100)
Event	Constant	Value	Real value
🏁 Reach goal	GOAL_REWARD	+10000	+100.00
👣 Normal step	STEP_REWARD	−10	−0.10
💥 Hit obstacle	OBSTACLE_REWARD	−1000	−10.00
🚧 Leave the grid	BOUNDARY_REWARD	−1000	−10.00

A blocked move keeps the robot in its current cell.

⚙️ Training parameters
Parameter	Value	Meaning
EPISODES	1000	Each starts at (0,0)
MAX_STEPS	100	Step limit per episode
EPSILON_THRES	20	20% random, 80% greedy
ALPHA_SCALED	15	α = 0.15
GAMMA_SCALED	90	γ = 0.90
SCALE_FACTOR	100	Fixed-point scale

An episode ends on reaching the goal, hitting an obstacle, or leaving the grid.

🚀 Getting Started

Prerequisites: a C compiler (GCC or Clang). No external libraries.

bash
# Build  (replace qroute.c with your actual file name)
gcc -std=c99 -Wall -Wextra -o qroute qroute.c

# Run
./qroute
🎛️ Menu
text
1. Display Environment Map
2. Train Robot
3. Display Learned Path
4. Display Memory Usage
5. Exit
Option	Action
1	Prints the 6×6 map
2	Runs 1000 episodes of epsilon-greedy Q-learning
3	Follows the greedy policy and marks the route with *, warning on a collision
4	Prints grid, Q-table and total bytes via sizeof()
5	Exits

⚠️ Tip: run option 2 before option 3. An untrained all-zero table always chooses UP.

Invalid input is caught by checking scanf, flushing the buffer and re-prompting.

<details> <summary><b>📟 Example memory report (option 4)</b></summary> <br>
text
Grid Memory: 144 bytes
Q-Table Memory (int16_t): 288 bytes
Total Data Memory: 432 bytes (Zero Dynamic Allocation)
</details>
🤖 AI-Assisted Development Log
1️⃣ Baselinefloat
2️⃣ Fixed-pointint16_t
3️⃣ Overflow fixint32_t math
4️⃣ Grid +rewards
5️⃣ Trainingloop
6️⃣ Saturationclamp
7️⃣ Menu +report
✅ 288-byteQ-table
#	Stage	Q-table RAM	Issue found	Fix carried forward
1	Baseline concept	576 B	Float bloat, FPU dependency	Move to int16_t fixed-point
2	Memory constraint	288 B	16-bit overflow, no learning rate	Use int32_t intermediates
3	Overflow resolution	288 B	None in single-update test	Build the environment
4	Grid and rewards	288 B	Nothing learns, always UP	Add training loop
5	Training loop	288 B	Unchecked narrowing, untuned α	Add clamp, tune α
6	Saturation safety	288 B	No UI, no memory report	Add menu and report
7	Menu and report	288 B	See limitations	n/a
<details> <summary><b>💬 Exact prompts used (stages 1–3)</b></summary> <br>

Prompt 1

"QRoute-C: A Memory-Constrained Q-Learning Framework for Autonomous Robot Path Planning"

Prompt 2

"Modify the framework to be strictly memory-constrained, zero dynamic memory allocation, and use integers instead of floats for microcontrollers."

Prompt 3

"Fix the integer overflow bug in the fixed-point Bellman equation calculation while keeping it strictly 16-bit storage."

The exact prompts for stages 4–7 are recorded in the project presentation.

</details>
⚠️ Known Limitations

Listed openly so results are not over-claimed:

🔸 Untrained policy: "Display Learned Path" before training follows an all-zero table (always UP).
🔸 EOF handling: while (getchar() != '\n') can hang if input reaches EOF.
🔸 Integer truncation: with α = 0.15, a TD error of 6 or less (0.06 real) gives a zero update.
🔸 Fixed map: the grid is hard-coded; there is no map loader.
🔸 Fixed-point only: sparse storage was considered at the start but is not implemented.
🔸 Evaluation scope: the 50% saving is measured with sizeof(); path quality is shown through the learned-path display, not a full benchmark.
🔸 Randomness: exploration uses rand() seeded by time(NULL), so runs vary.
📁 Project Structure
text
QRoute-C/
├── qroute.c          # Complete C source (rename to match your file)
├── README.md         # This file
└── docs/
    └── QRoute-C_AI-Assisted_Embedded_Q-Learning.pdf   # Project presentation


Built with ❤️ by Team 08 · UPES

Code generated and debugged iteratively with AI-assisted prompt engineering, then reviewed and corrected by the team.
