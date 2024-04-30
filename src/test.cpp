#include <curses.h>
#include <ncurses.h>
#include <string>

void printPWM(std::string direction) {
    clear(); // Clear the screen
    mvprintw(0, 0, "PWM Values:");
    mvprintw(5, 0, "%s", direction);
    refresh(); // Refresh the screen
}

int main() {
    while(1) {
        printPWM("Hello");
    }
}

