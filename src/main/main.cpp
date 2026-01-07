#include "Pendulum.hpp"
#include <ncurses.h>
#include <ratio>
#include <thread>
#include <chrono>
#include <string>

void draw_line(int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = -abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        // Use mvaddch to plot the character
        // Note: ncurses uses (y, x) order
        mvaddch(y1, x1, '.'); 

        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void draw_frame() {
    for (int a = 0; a < COLS; a ++)
        mvaddch(LINES-2, a, '-');

    // Print a message elsewhere
    mvprintw(LINES - 1, 0, "Press any key to exit ...");
}

void draw_pendulum(Pendulum pendulum, double relative_horizontal_position) {
    auto pivot = pendulum.getPivot();

    auto max_height = pivot.y * 2;
    auto max_width_from_center = pivot.y;

    int x_offset = COLS * relative_horizontal_position;

    const auto base_line = LINES - 2;

    int pivot_y = base_line - (pivot.y / max_height) * LINES;
    int pivot_x = (pivot.x / max_width_from_center) * COLS / 2 + x_offset; 

    for (int a = pivot_y+1; a < LINES - 2; a ++)
        mvaddch(a, pivot_x, '|');

    auto weight1 = pendulum.getWeight1();
    int weight_1_y = base_line - (weight1.y / max_height) * LINES;
    int weight_1_x = (weight1.x / max_width_from_center) * COLS * 0.6 / 2 + x_offset;
    auto weight2 = pendulum.getWeight2();
    int weight_2_y = base_line - (weight2.y / max_height) * LINES;
    int weight_2_x = (weight2.x / max_width_from_center) * COLS * 0.6 / 2 + x_offset;
    draw_line(weight_1_x, weight_1_y, weight_2_x, weight_2_y);
    draw_line(weight_1_x, weight_1_y, pivot_x, pivot_y);

    mvaddch(pivot_y, pivot_x, '#');
    mvaddch(weight_1_y, weight_1_x, 'O');
    mvaddch(weight_2_y, weight_2_x, 'O');
}

void print_energy(Pendulum pendulum, int position) {

    std::string l1 = std::format("Pendulum total energy: {}", std::round(pendulum.energy().total() * 100.0) / 100.0).c_str();
    std::string l2 = std::format("E kinetic: {}", std::round(pendulum.energy().kin * 100.0) / 100.0).c_str();
    std::string l3 = std::format("E potential: {}", std::round(pendulum.energy().pot * 100.0) / 100.0).c_str();

    int pos_x;
    if (position == 1) {
        pos_x = COLS - 30;
    } else {
        pos_x = 1;
    }

    mvprintw(1, pos_x, "%s", l1.c_str());
    mvprintw(2, pos_x, "%s", l2.c_str());
    mvprintw(3, pos_x, "%s", l3.c_str());

}

void draw(Pendulum pendulum_1, Pendulum pendulum_2) {
    // 1. Initialize the screen
    initscr();            
    raw();                // Disable line buffering (get input immediately)
    keypad(stdscr, TRUE); // Enable arrow keys/function keys
    noecho();             // Don't print the keys the user types
    curs_set(0);          // Hide the text cursor

    // The nodelay option causes getch to be a non-blocking call. 
    // If no input is ready, getch returns ERR.
    nodelay(stdscr, TRUE);

    int h_ms = 1;

    bool run = true;
    for (int a = 0; run; a++) {
        auto start = std::chrono::high_resolution_clock::now();
        if (a % 30 == 0) {
            clear();
            draw_frame();
            draw_pendulum(pendulum_1, 1/3.0);
            draw_pendulum(pendulum_2, 2/3.0);
            // print_energy(pendulum_1, 1);
            // print_energy(pendulum_2, 0);
            refresh();

            a = 0;
        }

        // 4. Wait for user input so the program doesn't close instantly
        if (getch() != ERR) run = false;

        if (false) {
            pendulum_1.calculateAndApplyForcesEulerCromer(h_ms);
            pendulum_2.calculateAndApplyForcesEulerCromer(h_ms);
        } else {
            pendulum_1.calculateAndApplyForcesRungeKutta(h_ms);
            pendulum_2.calculateAndApplyForcesRungeKutta(h_ms);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto sleep_duration = std::chrono::duration<int, std::milli>(h_ms) - (end - start);
        std::this_thread::sleep_for(sleep_duration);

    }

    // 5. De-initialize and return memory to the terminal
    endwin();
}

int main() {
    Pendulum pendulum_1;
    pendulum_1.init(120.0, 145.0);
    // pendulum.init(180.0, 180.0);

    Pendulum pendulum_2;
    pendulum_2.init(120.0, 146.0);
    // pendulum.init(180.0, 180.0);

    draw(pendulum_1, pendulum_2);

    return 0;
}
