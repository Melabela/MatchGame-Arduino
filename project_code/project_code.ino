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

bool DEBUG_PRINT = true;


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
    startup_checks();
    lcd.clear();
}


/* --------- Main Loop -------- */


void loop()
{
}

