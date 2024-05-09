#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define LCD_I2C_ADDR 0x27
#define LCD_I2C_COLS 16
#define LCD_I2C_ROWS 2

#define JOYSTICK_XAXIS_PIN A0
#define JOYSTICK_YAXIS_PIN A1

#define PUSH_BUTTON_PIN 2

// NOTE: using Servo library disables PWM on pins 9, 10
// NOTE: Servo library uses Timer1
#define SERVO_ROT_PIN 4

// NOTE: using tone() fn disables PWM on pins 3, 11
// NOTE: Servo library uses Timer2
#define BUZZER_PIN 7

#define SHIFT_REG_DATA_PIN   8
#define SHIFT_REG_LATCH_PIN  9
#define SHIFT_REG_CLOCK_PIN 10

#define RGBLED_R_PIN 11
#define RGBLED_G_PIN 12
#define RGBLED_B_PIN 13

// 16 cols x 2 rows
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_I2C_COLS, LCD_I2C_ROWS);

Servo servo;

// 8x Fixed-LEDs, and 7x gap positions in between = 15 total positions
// - center on 90 degrees; 
// - +/- 12 degrees per step;
// - descending angle order, to get desired rotation:
//     - left (low idx) = close to 9-oclock
//     - right (high idx) = close to 3-oclock
const byte servo_rotation_angles[] = {
    174, 162, 150, 138, 126,
    114, 102,  90,  78,  66,
     54,  42,  30,  18,   6
};

const byte MAX_SERVO_ROT_POSN = sizeof(servo_rotation_angles) - 1;

// bit[0] = red
// bit[1] = green
// bit[2] = blue
enum rgbled_color_e
{
    RGBLED_OFF = 0,
    RGBLED_RED = 1,
    RGBLED_GREEN = 2,
    RGBLED_YELLOW,
    RGBLED_BLUE = 4,
    RGBLED_MAGENTA,
    RGBLED_CYAN,
    RGBLED_WHITE = 7
};

const byte MAX_RGBLED_COLOR = RGBLED_WHITE;

const byte rgbled_user_colors[] = {
    RGBLED_RED,
    RGBLED_YELLOW,
    RGBLED_GREEN,
    RGBLED_BLUE,
};

const byte MAX_RGBLED_USER_COLORS = sizeof(rgbled_user_colors) - 1;

/* Board setup, in rainbow color order */
const byte fixed_leds_colors[] = {
    RGBLED_RED,
    RGBLED_YELLOW,
    RGBLED_GREEN,
    RGBLED_BLUE,
    RGBLED_RED,
    RGBLED_YELLOW,
    RGBLED_GREEN,
    RGBLED_BLUE,
};

const byte NUM_FIXED_LEDS = sizeof(fixed_leds_colors);


enum tones_e
{
    TONE_OFF = -1,
    // -- idx[0] --
    TONE_A2 = 0,  // 110Hz
    TONE_A2_sh,
    TONE_B2,
    TONE_C3,
    TONE_C3_sh,
    TONE_D3,
    TONE_D3_sh,
    TONE_E3,
    TONE_F3,
    TONE_F3_sh,
    TONE_G3,
    TONE_G3_sh,
    // -- idx[12] --
    TONE_A3, // 220Hz
    TONE_A3_sh,
    TONE_B3,
    TONE_C4,
    TONE_C4_sh,
    TONE_D4,
    TONE_D4_sh,
    TONE_E4,
    TONE_F4,
    TONE_F4_sh,
    TONE_G4,
    TONE_G4_sh,
    // -- idx[24] --
    TONE_A4, // 440Hz
};

word tone_hz[] = {
    // -- idx[0] --
    110,  // A2
    117,  // A2_sh
    123,  // B2
    131,  // C3
    139,  // C3_sh
    147,  // D3
    156,  // D3_sh
    165,  // E3
    175,  // F3
    185,  // F3_sh
    196,  // G3
    208,  // G3_sh
    // -- idx[12] --
    220,  // A3
    233,  // A3_sh
    247,  // B3
    262,  // C4
    277,  // C4_sh
    294,  // D4
    311,  // D4_sh
    330,  // E4
    349,  // F4
    370,  // F4_sh
    392,  // G4
    415,  // G4_sh
    // -- idx[24] --
    440,  // A4
};


/* --------- Global Variables -------- */

/* minimum (absolute) value to activate */
#define JOYSTICK_REL_MIN_THRESH 300

/* frames to not-apply input in held,
 *  to make movements/changes more discrete */
#define JOYSTICK_X_COOLDOWN_FRAMES 6
#define JOYSTICK_Y_COOLDOWN_FRAMES 10

#define GAME_START_TIME_SEC 30
#define GAME_FRAMES_PER_SEC 60

#define MILLIS_PER_FRAME 16

bool DEBUG_PRINT = true;

enum game_states_e
{
    // post-setup() first state
    GAME_ST_NONE = 0,
    // waiting for user to start game
    GAME_ST_WAIT_START,
    // initiate game
    GAME_ST_START_GAME,
    // new round: pick fixed leds
    GAME_ST_ROUND_NEW,
    // round in progress: user matching
    GAME_ST_ROUND_IN_PROG,
    // round done: award points, etc.
    GAME_ST_ROUND_DONE,
    // game done: time expired
    GAME_ST_END_GAME,
};

const char* game_state_names[] = {
    "GAME_ST_NONE",
    "GAME_ST_WAIT_START",
    "GAME_ST_START_GAME",
    "GAME_ST_ROUND_NEW",
    "GAME_ST_ROUND_IN_PROG",
    "GAME_ST_ROUND_DONE",
    "GAME_ST_END_GAME",
};

int game_state_last = 0;
int game_state = 0;

int game_user_position = 0;
int game_user_color_idx = 0;

int game_frames_remain = 0;
int game_score = 0;

byte game_fixed_leds_mask_on = 0;


bool button_pressed_last = false;
bool button_pressed = false;

// joystick value after dired to get simple
//  +1 / 0 / -1, for (up,right) / middle / (down,left)
int joystick_x_dir_last = 0;
int joystick_x_dir = 0;

int joystick_y_dir_last = 0;
int joystick_y_dir = 0;

unsigned long millis_at_last_frame = 0;


/* --------- Component Helper Fns -------- */

// val_min & val_max are inclusive
int cap_value(int val, int val_min, int val_max)
{
    return (val < val_min) ? val_min :
           (val > val_max) ? val_max : val;
}


// is value within range?
// - val_min & val_max are inclusive
bool is_value_within(int val, int val_min, int val_max)
{
    return ((val >= val_min) && (val <= val_max));
}


void rgbled_set_color(int color_val)
{
    if (DEBUG_PRINT)
    {
        Serial.print("rgbled_set_color: color_val=0x");
        Serial.println(color_val, HEX);
    }

    if (is_value_within(color_val, RGBLED_OFF, RGBLED_WHITE))
    {
        // RGBLED_* enums are arranged to match bit combinations
        //  - bit[0] = red;  bit[1] = green;  bit[2] = blue
        int r, g, b;
        r = (color_val & 0x1) ? HIGH : LOW;
        g = (color_val & 0x2) ? HIGH : LOW;
        b = (color_val & 0x4) ? HIGH : LOW;

        // use positive logic, drive power on color pins
        //  since RGBLED is common Cathode
        digitalWrite(RGBLED_R_PIN, r);
        digitalWrite(RGBLED_G_PIN, g);
        digitalWrite(RGBLED_B_PIN, b);
    }
}


void servo_set_rotation(int position)
{
    if (is_value_within(position, 0, MAX_SERVO_ROT_POSN))
    {
        byte angle = servo_rotation_angles[position];

        if (DEBUG_PRINT)
        {
            Serial.print("servo_set_rotation(): position=");
            Serial.print(position);
            Serial.print(", angle=");
            Serial.println(angle);
        }

        servo.write(angle);
    }
}


void buzzer_play_tone(int tone_idx, int tone_msec)
{
    if (DEBUG_PRINT)
    {
        Serial.print("buzzer_play_tone(): idx=");
        Serial.print(tone_idx);
        Serial.print(", msec=");
        Serial.println(tone_msec);
    }

    if (tone_idx <= TONE_OFF)
    {
        noTone(BUZZER_PIN);
    }

    if (is_value_within(tone_idx, TONE_A2, TONE_A4))
    {
        int hz = tone_hz[tone_idx];
        tone(BUZZER_PIN, hz, tone_msec);
    }
}


void fixed_leds_set(byte leds_bitmask)
{
    if (DEBUG_PRINT)
    {
        Serial.print("fixed_leds_set(): bitmask=0x");
        Serial.println(leds_bitmask, HEX);
    }

    digitalWrite(SHIFT_REG_LATCH_PIN, LOW);
    shiftOut(SHIFT_REG_DATA_PIN,
             SHIFT_REG_CLOCK_PIN,
             MSBFIRST, leds_bitmask);
    digitalWrite(SHIFT_REG_LATCH_PIN, HIGH);
}


void button_read()
{
    // Button Input
    // - open: pulled-up internlly (HIGH)
    // - closed: driven to Ground (LOW)
    int button_raw = digitalRead(PUSH_BUTTON_PIN);

    if (0)  // (DEBUG_PRINT)
    {
        Serial.print("button_read: raw_val=");
        Serial.println(button_raw);
    }

    // save last state, before updating
    button_pressed_last = button_pressed;

    button_pressed = !button_raw;
}


void joystick_read_x_y()
{
    int joystick_x_raw = analogRead(JOYSTICK_XAXIS_PIN);
    int joystick_y_raw = analogRead(JOYSTICK_YAXIS_PIN);

    // joystick Y raw reading seems inverted
    //  (relative to X's left/right), so flip value
    joystick_y_raw = 1023 - joystick_y_raw;

    if (0)  // (DEBUG_PRINT)
    {
        Serial.print("joystick_x: raw_val=");
        Serial.println(joystick_x_raw);
        Serial.print("joystick_y: raw_val=");
        Serial.println(joystick_y_raw);
    }

    // adjust so middle position is zero
    int joystick_x_rel = joystick_x_raw - 512;
    int joystick_y_rel = joystick_y_raw - 512;

    // save last state, before updating
    joystick_x_dir_last = joystick_x_dir;
    joystick_y_dir_last = joystick_y_dir;

    // apply a dir to get simple +1 / 0 / -1,
    //  for (up,right) / middle / (down,left)
    joystick_x_dir =
            (joystick_x_rel < -JOYSTICK_REL_MIN_THRESH) ? -1 :
            (joystick_x_rel > +JOYSTICK_REL_MIN_THRESH) ? +1 : 0;
    joystick_y_dir =
            (joystick_y_rel < -JOYSTICK_REL_MIN_THRESH) ? -1 :
            (joystick_y_rel > +JOYSTICK_REL_MIN_THRESH) ? +1 : 0;
}


/* --------- Setup -------- */

void init_IOs()
{
    if (DEBUG_PRINT)
    {
        Serial.println("init_IOs()");
    }

    // LCD I2C init
    lcd.init();
    lcd.clear();
    lcd.backlight();

    // Push-button setup
    pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

    // Servo setup
    servo.attach(SERVO_ROT_PIN);

    // Passive Buzzer setup
    pinMode(BUZZER_PIN, OUTPUT);

    // Shift Register (74HC595) setup
    pinMode(SHIFT_REG_DATA_PIN,  OUTPUT);
    pinMode(SHIFT_REG_LATCH_PIN, OUTPUT);
    pinMode(SHIFT_REG_CLOCK_PIN, OUTPUT);

    // RGBLED setup
    pinMode(RGBLED_R_PIN, OUTPUT);
    pinMode(RGBLED_G_PIN, OUTPUT);
    pinMode(RGBLED_B_PIN, OUTPUT);
}


void startup_checks()
{
    if (DEBUG_PRINT)
    {
        Serial.println("startup_checks()");
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Startup checks");

    // Test Fixed LEDs
    lcd.setCursor(0, 1);
    lcd.print("- Fixed LEDs    ");

    byte bitmask = 0x88;
    for (int i=0; i<4; i++)
    {
        fixed_leds_set(bitmask);
        delay(500);
        bitmask >>= 1;
    }
    fixed_leds_set(0x00);

    // Test Servo Positions
    lcd.setCursor(0, 1);
    lcd.print("- Servo rotation");

    for (int pos=0; pos<=MAX_SERVO_ROT_POSN; pos++)
    {
        servo_set_rotation(pos);
        delay(500);
    }

    // Test RGB_LED
    lcd.setCursor(0, 1);
    lcd.print("- RGB LED       ");

    for (byte color=RGBLED_RED; color<=RGBLED_WHITE; color++)
    {
        rgbled_set_color(color);
        delay(500);
    }
    rgbled_set_color(RGBLED_OFF);

    // Test Buzzer
    lcd.setCursor(0, 1);
    lcd.print("- Buzzer        ");
    for (int note=TONE_A2; note<=TONE_A4; note++)
    {
        buzzer_play_tone(note, 200);
        delay(200);
    }
}


void setup()
{
    Serial.begin(115200);
    init_IOs();
    // startup_checks();
    lcd.clear();
    delay(500);
}


/* --------- Main Loop -------- */

int game_state_wait_start()
{
    /* NONE / END_GAME -> WAIT_START */
    if ((game_state_last == GAME_ST_NONE) ||
        (game_state_last == GAME_ST_END_GAME))
    {
        // update display message, tell user to start
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Press Button");
        lcd.setCursor(0, 1);
        lcd.print(" to Start game.");

        // reset button stored states
        button_pressed = button_pressed_last = 0;
        return GAME_ST_WAIT_START;
    }

    /* WAIT_START -> WAIT_START */
    else if (game_state_last == GAME_ST_WAIT_START)
    {
        // accept button press to start
        button_read();
        if (button_pressed && !button_pressed_last)
        {
            // move to next state
            return GAME_ST_START_GAME;
        }
    }

    return GAME_ST_WAIT_START;
}


int game_state_start_game()
{
    /* WAIT_START -> START_GAME */
    if (game_state_last == GAME_ST_WAIT_START)
    {
        // rotation: start in middle position
        game_user_position = MAX_SERVO_ROT_POSN / 2;
        servo_set_rotation(game_user_position);

        // RGB LED: start with "off" color
        game_user_color_idx = -1;
        rgbled_set_color(RGBLED_OFF);

        // set time & score
        game_frames_remain = GAME_START_TIME_SEC * GAME_FRAMES_PER_SEC;
        game_score = 0;

        // move to next state
        return GAME_ST_ROUND_NEW;
    }

    return GAME_ST_START_GAME;
}


int game_state_round_new()
{
    /* START_NEW / ROUND_DONE -> ROUND_NEW */
    if ((game_state_last == GAME_ST_START_GAME) ||
        (game_state_last == GAME_ST_ROUND_DONE))
    {
        // randomly pick a few Fixed LEDs to enable
        game_fixed_leds_mask_on = 0x00;

        byte num_leds = random(1, 4);  // [1..3]
        byte n_chosen = 0;
        while(n_chosen < num_leds)
        {
            byte led_choice = random(0, NUM_FIXED_LEDS);
            byte choice_bit = (1 << led_choice);
            if ( !(game_fixed_leds_mask_on & choice_bit) )
            {
                game_fixed_leds_mask_on |= choice_bit;
                n_chosen++;
            }
        }

        fixed_leds_set(game_fixed_leds_mask_on);

        // set LCD fixed text
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Time Left: ");
        lcd.setCursor(0, 1);
        lcd.print("Score: ");

        // move to next state
        return GAME_ST_ROUND_IN_PROG;
    }

    return GAME_ST_ROUND_NEW;
}


void display_update_time_score()
{
    // NO clear() to avoid flicker, and update the minimum text needed
    // - FYI labels per line, were set in game_state_round_new()

    int time_secs_left = game_frames_remain / GAME_FRAMES_PER_SEC;
    int tenth_sec_left = (game_frames_remain % GAME_FRAMES_PER_SEC) / 6;

    lcd.setCursor(11, 0);
    if (time_secs_left < 10)
    {
        lcd.print(' ');
    }
    lcd.print(time_secs_left);
    lcd.print('.');
    lcd.print(tenth_sec_left);

    lcd.setCursor(7, 1);
    lcd.print(game_score);
}


void check_for_match()
{
}


void handle_inputs_round()
{
    /* add counters so if joystick input is held,
     *  we do not apply its input every frame */
    static int joy_x_dir_cooldown = 0;
    static int joy_y_dir_cooldown = 0;

    /* Check inputs & act (in priority order)
     * ----
     * 1. button_pressed:
     *    - check match at current position/color
     *    - (ignore joystick inputs)
     *
     * 2a. joystick_x:
     *     - change rotation position
     * 2b. joystick_y:
     *     - change RGB_LED color
     */

    button_read();
    if (button_pressed)
    {
        check_for_match();
    }
    else
    {
        joystick_read_x_y();

        /* change rotation position */
        if (joystick_x_dir != 0)
        {
            bool apply_movement = false;

            if (joystick_x_dir != joystick_x_dir_last)
            {
                // first frame joystick_x_dir changed
                apply_movement = true;
            }
            else if (--joy_x_dir_cooldown <= 0)
            {
                // dir held, and cooldown over
                apply_movement = true;
            }

            if (apply_movement)
            {
                int user_pos = game_user_position;
                user_pos += joystick_x_dir;
                // bound at left/right ends, can't turn further
                user_pos = cap_value(user_pos, 0, MAX_SERVO_ROT_POSN);

                game_user_position = user_pos;
                servo_set_rotation(game_user_position);

                // set cooldown again, in case still held
                joy_x_dir_cooldown = JOYSTICK_X_COOLDOWN_FRAMES;
            }
        }

        /* change rgbled color */
        if (joystick_y_dir != 0)
        {
            bool apply_color = false;

            if (joystick_y_dir != joystick_y_dir_last)
            {
                // first frame joystick_y_dir changed
                apply_color = true;
            }
            else if (--joy_y_dir_cooldown <= 0)
            {
                // dir held, and cooldown over
                apply_color = true;
            }

            if (apply_color)
            {
                int user_color_idx = game_user_color_idx;
                user_color_idx += joystick_y_dir;

                // allow value roll-over on both ends
                if (user_color_idx < 0)
                {
                    user_color_idx = MAX_RGBLED_USER_COLORS;
                }
                else if (user_color_idx > MAX_RGBLED_USER_COLORS)
                {
                    user_color_idx = 0;
                }

                game_user_color_idx = user_color_idx;
                int color_val = rgbled_user_colors[game_user_color_idx];
                rgbled_set_color(color_val);

                // set cooldown, delay apply if dir is held
                joy_y_dir_cooldown = JOYSTICK_Y_COOLDOWN_FRAMES;
            }
        }
    }
}


int game_state_round_in_prog()
{
    display_update_time_score();

    handle_inputs_round();

    return GAME_ST_ROUND_IN_PROG;
}


int game_state_round_done()
{
    return GAME_ST_ROUND_DONE;
}


int game_state_end_game()
{
    static int game_end_wait_frames = 0;

    /* Can come from most other states,
        since move here on time_remain empty */
    if (game_state_last != GAME_ST_END_GAME)
    {
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("Time Up...");
        lcd.setCursor(2, 1);
        lcd.print("Game Over.");

        // setup wait counter, to hold & display above msg
        game_end_wait_frames = 6 * GAME_FRAMES_PER_SEC;
    }

    if (game_state_last == GAME_ST_END_GAME)
    {
        game_end_wait_frames--;
        if (game_end_wait_frames == (3 * GAME_FRAMES_PER_SEC))
        {
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Game Over.");
            lcd.setCursor(0, 1);
            lcd.print("Your Score: ");
            lcd.print(game_score);
        }
        else if (game_end_wait_frames <= 0)
        {
            // move back to pre-game, wait to start again
            return GAME_ST_WAIT_START;
        }
    }

    return GAME_ST_END_GAME;
}


void process_frame()
{
    int state_next;

    if (game_state_last != game_state)
    {
        Serial.println("start process_frame()");
        Serial.print(" - game_state (last -> now): ");
        Serial.print(game_state_names[game_state_last]);
        Serial.print(" -> ");
        Serial.println(game_state_names[game_state]);
    }

    switch (game_state)
    {
    case GAME_ST_NONE:
        state_next = GAME_ST_WAIT_START;
        break;
    case GAME_ST_WAIT_START:
        state_next = game_state_wait_start();
        break;
    case GAME_ST_START_GAME:
        state_next = game_state_start_game();
        break;
    case GAME_ST_ROUND_NEW:
        state_next = game_state_round_new();
        break;
    case GAME_ST_ROUND_IN_PROG:
        state_next = game_state_round_in_prog();
        break;
    case GAME_ST_ROUND_DONE:
        state_next = game_state_round_done();
        break;
    case GAME_ST_END_GAME:
        state_next = game_state_end_game();
        break;
    };

    /* capture current state -> last */
    game_state_last = game_state;
    /* update next_state -> current */
    game_state = state_next;

    /* count down frames if game active */
    if (game_state > GAME_ST_START_GAME)
    {
        if (--game_frames_remain <= 0)
        {
            game_state = GAME_ST_END_GAME;
        }
    }
}


void loop()
{
    unsigned long millis_now = millis();
    long millis_delta = millis_now - millis_at_last_frame;

    if (millis_delta > MILLIS_PER_FRAME)
    {
        millis_at_last_frame = millis_now;
        process_frame();
    }
    else
    {
        delay(2);
    }
}

