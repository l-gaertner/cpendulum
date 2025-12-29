#include "Pendulum.hpp"
#include <ncurses.h>
#include <thread>
#include <chrono>

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

void draw(Pendulum pendulum) {
    // 1. Initialize the screen
    initscr();            
    raw();                // Disable line buffering (get input immediately)
    keypad(stdscr, TRUE); // Enable arrow keys/function keys
    noecho();             // Don't print the keys the user types
    curs_set(0);          // Hide the text cursor

    // The nodelay option causes getch to be a non-blocking call. 
    // If no input is ready, getch returns ERR.
    nodelay(stdscr, TRUE);

    auto pivot = pendulum.getPivot();

    auto max_height = pivot.y * 2;
    auto max_width_from_center = pivot.y;

    int pivot_y = (pivot.y / max_height) * LINES;
    int pivot_x = (pivot.x / max_width_from_center) * COLS / 2 + COLS / 2;

    bool run = true;
    for (int a = 0; run; a++) {
        // 2. Draw the character
        // move(y, x) then addch(char)
        // Note: ncurses uses (y, x) order, not (x, y)

        for (int a = pivot_y+1; a < LINES - 1; a ++)
            mvaddch(a, pivot_x, '|');

        for (int a = 0; a < COLS; a ++)
            mvaddch(LINES-2, a, '-');

        auto weight1 = pendulum.getWeight1();
        int weight_1_y = (weight1.y / max_height) * LINES;
        int weight_1_x = (weight1.x / max_width_from_center) * COLS * 0.6 / 2 + COLS / 2;

        auto weight2 = pendulum.getWeight2();
        int weight_2_y = (weight2.y / max_height) * LINES;
        int weight_2_x = (weight2.x / max_width_from_center) * COLS * 0.6 / 2 + COLS / 2;

        draw_line(weight_1_x, weight_1_y, weight_2_x, weight_2_y);
        draw_line(weight_1_x, weight_1_y, pivot_x, pivot_y);

        mvaddch(pivot_y, pivot_x, '#');
        mvaddch(weight_1_y, weight_1_x, 'O');
        mvaddch(weight_2_y, weight_2_x, 'O');

        // Print a message elsewhere
        mvprintw(LINES - 1, 0, "Press any key to exit ...");

        // 3. Refresh the screen to show the changes
        refresh();

        // 4. Wait for user input so the program doesn't close instantly
        if (getch() != ERR) run = false;

        auto loop_duration = std::chrono::milliseconds(1000/60);
        std::this_thread::sleep_for(loop_duration);

        pendulum.calculateAndApplyForces(1000/60);

        clear();
    }

    // 5. De-initialize and return memory to the terminal
    endwin();
}

int main() {
    Pendulum pendulum;
    pendulum.init(120.0, 145.0);

    draw(pendulum);

    return 0;
}
