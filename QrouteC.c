/**
 * @file qroute.c
 * @brief QRoute-C: A Memory-Constrained Q-Learning Framework for Autonomous Robot Path Planning
 * @details Implements fixed-point int16_t Q-learning with zero dynamic memory allocation, 
 *          32-bit overflow-safe Bellman updates, full training episodes, rewards, and autonomous path generation.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define GRID_SIZE 6
#define NUM_STATES (GRID_SIZE * GRID_SIZE) // 36 states
#define NUM_ACTIONS 4                      // 0: Up, 1: Down, 2: Left, 3: Right

#define EPISODES 1000
#define MAX_STEPS 100

// Action definitions
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3

// Fixed-point scaled rewards (Scaled by 100 to match fixed-point arithmetic)
#define GOAL_REWARD 10000     // +100.00
#define STEP_REWARD -10       // -0.10
#define OBSTACLE_REWARD -1000 // -10.00
#define BOUNDARY_REWARD -1000 // -10.00

// Fixed-point scaling parameters (Alpha = 0.15, Gamma = 0.90, Epsilon = 0.20)
#define ALPHA_SCALED 15
#define GAMMA_SCALED 90
#define SCALE_FACTOR 100
#define EPSILON_THRES 20      // Out of 100 for random exploration

// Grid representation:
// 0 = Empty cell
// 1 = Obstacle / Wall
// 2 = Starting position (0,0)
// 3 = Goal position (5,5)
int grid[GRID_SIZE][GRID_SIZE] = {
    {2, 0, 0, 0, 0, 0},
    {0, 1, 1, 0, 1, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 1, 1, 0, 0, 0},
    {0, 0, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 3}
};

// Q-Agent structure using strictly static allocation (No heap usage)
typedef struct {
    int16_t q_table[NUM_STATES][NUM_ACTIONS]; // 36 * 4 * 2 bytes = 288 bytes total
} QAgent;

/**
 * @brief Maps 2D grid coordinates to a 1D state index.
 */
int get_state_index(int x, int y) {
    return y * GRID_SIZE + x;
}

/**
 * @brief Initialize all Q-values to zero.
 */
void initialize_q_table(QAgent *agent) {
    memset(agent->q_table, 0, sizeof(agent->q_table));
}

/**
 * @brief Find the highest Q-value for a given (x, y) state.
 */
int16_t get_max_q_value(QAgent *agent, int x, int y) {
    int state = get_state_index(x, y);
    int16_t max_val = agent->q_table[state][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (agent->q_table[state][a] > max_val) {
            max_val = agent->q_table[state][a];
        }
    }
    return max_val;
}

/**
 * @brief Find the best action for a given (x, y) state based on current Q-table.
 */
int get_max_action(QAgent *agent, int x, int y) {
    int state = get_state_index(x, y);
    int best_action = 0;
    int16_t max_val = agent->q_table[state][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (agent->q_table[state][a] > max_val) {
            max_val = agent->q_table[state][a];
            best_action = a;
        }
    }
    return best_action;
}

/**
 * @brief Select action using epsilon-greedy exploration strategy.
 */
int choose_action(QAgent *agent, int x, int y) {
    if ((rand() % 100) < (int)(EPSILON_THRES)) {
        return rand() % ACTIONS; // Exploration
    }
    return get_max_action(agent, x, y); // Exploitation
}

/**
 * @brief Move the robot and return the reward, updating coordinates.
 */
int move_robot(int *x, int *y, int action) {
    int new_x = *x;
    int new_y = *y;

    if (action == UP) new_y--;
    else if (action == DOWN) new_y++;
    else if (action == LEFT) new_x--;
    else if (action == RIGHT) new_x++;

    // Check boundaries
    if (new_x < 0 || new_x >= GRID_SIZE || new_y < 0 || new_y >= GRID_SIZE) {
        return BOUNDARY_REWARD;
    }

    // Check obstacles
    if (grid[new_y][new_x] == 1) {
        return OBSTACLE_REWARD;
    }

    // Update position
    *x = new_x;
    *y = new_y;

    // Check goal
    if (grid[new_y][new_x] == 3) {
        return GOAL_REWARD;
    }

    return STEP_REWARD;
}

/**
 * @brief Overflow-safe Bellman update using 32-bit intermediate casting and int16_t fixed-point storage.
 */
void update_q_value(QAgent *agent, int state, int action, int reward, int next_state) {
    int16_t max_next_q = agent->q_table[next_state][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (agent->q_table[next_state][a] > max_next_q) {
            max_next_q = agent->q_table[next_state][a];
        }
    }

    // 32-bit promotion preventing overflow
    int32_t future_scaled = ((int32_t)GAMMA_SCALED * (int32_t)max_next_q) / SCALE_FACTOR;
    int32_t target = (int32_t)reward + future_scaled;

    int32_t current_q = (int32_t)agent->q_table[state][action];
    int32_t diff = target - current_q;
    
    int32_t adjustment = (ALPHA_SCALED * diff) / SCALE_FACTOR;
    int32_t new_q = current_q + adjustment;

    // Narrowing bounds
    if (new_q > 32767) new_q = 32767;
    if (new_q < -32768) new_q = -32768;

    agent->q_table[state][action] = (int16_t)new_q;
}

/**
 * @brief Full Q-learning training loop over multiple episodes.
 */
void train_robot(QAgent *agent) {
    initialize_q_table(agent);
    printf("\n[TRAINING] Starting Q-Learning over %d episodes...\n", EPISODES);

    for (int ep = 1; ep <= EPISODES; ep++) {
        int x = 0, y = 0; // Start at (0,0)

        for (int step = 0; step < MAX_STEPS; step++) {
            int current_state = get_state_index(x, y);
            int action = choose_action(agent, x, y);

            int prev_x = x;
            int prev_y = x; // Just for tracking
            int reward = move_robot(&x, &y, action);
            int next_state = get_state_index(x, y);

            // Fixed-point Bellman update (Bug fixed: applied to current state & action)
            update_q_value(agent, current_state, action, reward, next_state);

            if (reward == GOAL_REWARD || reward == OBSTACLE_REWARD || reward == BOUNDARY_REWARD) {
                break;
            }
        }

        if (ep % 200 == 0) {
            printf("[PROGRESS] Episode %d / %d completed.\n", ep, EPISODES);
        }
    }
    printf("[SUCCESS] Training completed successfully using fixed-point int16_t math!\n");
}

/**
 * @brief Display the environment map.
 */
void display_map(void) {
    printf("\n--- QRoute-C Environment Map (6x6) ---\n");
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == 2) printf(" S ");
            else if (grid[y][x] == 3) printf(" G ");
            else if (grid[y][x] == 1) printf(" # ");
            else printf(" . ");
        }
        printf("\n");
    }
    printf("Legend: S = Start | G = Goal | # = Obstacle | . = Empty\n");
}

/**
 * @brief Display the autonomous path learned by the robot (`*`).
 */
void display_learned_path(QAgent *agent) {
    int x = 0, y = 0;
    int path[GRID_SIZE][GRID_SIZE] = {0};
    path[y][x] = 1;

    printf("\n--- Autonomous Learned Path ---\n");
    for (int step = 0; step < MAX_STEPS; step++) {
        int action = get_max_action(agent, x, y);
        int reward = move_robot(&x, &y, action);

        if (reward == OBSTACLE_REWARD || reward == BOUNDARY_REWARD) {
            printf("[WARNING] Policy hit obstacle/boundary at (%d,%d). Training needed?\n", x, y);
            break;
        }

        path[y][x] = 1;

        if (reward == GOAL_REWARD) {
            break;
        }
    }

    // Print grid with path
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == 2) printf(" S ");
            else if (grid[y][x] == 3) printf(" G ");
            else if (grid[y][x] == 1) printf(" # ");
            else if (path[y][x] == 1) printf(" * ");
            else printf(" . ");
        }
        printf("\n");
    }
    printf("Legend: S = Start | G = Goal | # = Obstacle | * = Learned Path\n");
}

/**
 * @brief Display memory footprint analysis.
 */
void display_memory_usage(QAgent *agent) {
    printf("\n--- Memory Footprint Analysis ---\n");
    printf("Grid Memory: %zu bytes\n", sizeof(grid));
    printf("Q-Table Memory (int16_t): %zu bytes\n", sizeof(agent->q_table));
    printf("Total Data Memory: %zu bytes (Zero Dynamic Allocation)\n", sizeof(grid) + sizeof(agent->q_table));
    printf("States: %d | Actions: %d | Total Q-Values: %d\n", NUM_STATES, NUM_ACTIONS, NUM_STATES * NUM_ACTIONS);
}

int main(void) {
    QAgent agent;
    srand((unsigned int)time(NULL));
    initialize_q_table(&agent);

    printf("====================================================\n");
    printf(" QRoute-C: Memory-Constrained Q-Learning Framework  \n");
    printf("====================================================\n");

    int choice = 0;
    do {
        printf("\n================ MENU ================\n");
        printf("1. Display Environment Map\n");
        printf("2. Train Robot (Q-Learning Episodes & Rewards)\n");
        printf("3. Display Autonomous Learned Path (*)\n");
        printf("4. Display Q-Table Memory & Stats (288 bytes)\n");
        printf("5. Exit Simulation\n");
        printf("Enter your choice (1-5): ");

        if (scanf("%d", &choice) != 1) {
            printf("Invalid input. Please enter 1-5.\n");
            while(getchar() != '\n');
            continue;
        }

        switch (choice) {
            case 1:
                display_map();
                break;
            case 2:
                train_robot(&agent);
                break;
            case 3:
                display_learned_path(&agent);
                break;
            case 4:
                display_memory_usage(&agent);
                break;
            case 5:
                printf("\nExiting QRoute-C simulation. Good luck with your hackathon presentation!\n");
                break;
            default:
                printf("[ERROR] Invalid choice. Select 1-5.\n");
        }
    } while (choice != 5);

    return 0;
}
