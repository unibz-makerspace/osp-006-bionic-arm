
/*

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


    Copyright (C) Lorenzo Miori lorenzo.miori[at]gmail[dot]com

    #    #   ##   #    # ###### #####   ####  #####    ##    ####  ###### 
    ##  ##  #  #  #   #  #      #    # #      #    #  #  #  #    # #      
    # ## # #    # ####   #####  #    #  ####  #    # #    # #      #####  
    #    # ###### #  #   #      #####       # #####  ###### #      #      
    #    # #    # #   #  #      #   #  #    # #      #    # #    # #      
    #    # #    # #    # ###### #    #  ####  #      #    #  ####  ###### 

    http://makerspace.inf.unibz.it/

    learning by doing

    ! Come and visit us !

*/

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

#include <SPI.h>              // We use this library, so it must be called here.
#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.

/************************/
/*     DEFINITIONS      */
/************************/

#define USE_IO_EXPANDER_BUTTONS

#ifdef USE_IO_EXPANDER_BUTTONS
#define BUTTON_START_PIN    8U             /**< Button start PIN [#] -> IO Expander Button S1 -> Swing arm */
#else
#define BUTTON_START_PIN    7U
#endif
#define DEBOUNCE_BUTTONS    100U * 1000U   /**< Button debounce time [us] */

/**< Enumerations for the logic state machine */
typedef enum
{

    ARM_NONE,
    ARM_STARTUP,
    ARM_WAIT_BUTTON,
    ARM_HAND_OPEN,
    ARM_CLOSE,
    ARM_HAND_CLOSE,
    ARM_ROTATE,
    ARM_ROTATE_BACK,
    ARM_OPEN,
    ARM_HAND_FINAL_OPEN

} e_arm_state;

/**< Enumeration of buttons */
enum e_buttons_
{

    BUTTON_START, // io expander button S1 -> swing arm
#ifdef USE_IO_EXPANDER_BUTTONS
	BUTTON_HAND_OPEN, // S2
	BUTTON_HAND_CLOSE, // S3
#endif

    NUM_BUTTONS

};

/**< Enumeration of directions */
enum e_direction
{

    MOV_IDLE,
    MOV_OPEN,
    MOV_CLOSE,

    NUM_DIRECTIONS

};

/************************/
/*      STRUCTURES      */
/************************/

/**< Keyboard status */
typedef struct _t_keypad
{

    bool       input[NUM_BUTTONS];
    uint32_t   debounce[NUM_BUTTONS];
    bool       latches[NUM_BUTTONS];
    bool       buttons[NUM_BUTTONS];

} t_keypad;

/** Rotation movement configuration */
typedef struct _t_rotation_movement
{

    uint8_t  angle_closed;         /**< Swing start agular position [°] */
    uint8_t  angle_open;           /**< Swing end agular position [°] */
    uint8_t  angle_step;           /**< Swing angular step [°] */
    uint32_t angle_step_delay;     /**< Arm swing movement step delay [us] */
  
} t_rotation_movement;

/** Structure holding configuration angles */
typedef struct _t_arm_config
{

    t_rotation_movement swing;      /**< Arm swing movement */    
    t_rotation_movement rotation;   /**< Arm rotation movement */
    t_rotation_movement hand;       /**< Arm hand movement */

} t_arm_config;

/** Structure holding the status of the associated servo motor */
typedef struct _t_servo_movement
{

    uint8_t  step_val;        /**< The angular step that is added/subracted every step_delay [°] */
    uint32_t step_delay;      /**< The delay between each step [us] */
    uint32_t start;           /**< The start timestamp [us] */
    uint8_t  angle_position;  /**< Current angular position [°] */
    uint8_t  angle_end;       /**< Destination angular position [°] */

} t_servo_movement;

/************************/
/*      GLOBALS         */
/************************/

static t_keypad keypad;            /**< Keypad driver status */

static Servo arm_swing_servo;      /**< Servo motor control for the arm swing position */
static Servo arm_rotation_servo;   /**< Servo motor control for the arm rotation position */
static t_arm_config arm_config;    /**< Arm configuration (angles and timings) */

static t_servo_movement arm_swing;     /**< Status of the arm swing servo movement */
static t_servo_movement arm_rotation;  /**< Status of the arm rotation servo movement */
static t_servo_movement hand;          /**< Status of the hand simple motor movement */

static MCP23S17 ioExpander(&SPI, 10, 0);

/************************/
/*     DECLARATIONS     */
/************************/

/* Function declarations */
void arm_init(t_arm_config *config, Servo swing_servo, Servo rotation_servo, t_servo_movement *arm_swing, t_servo_movement *arm_rotation, t_servo_movement *hand);
void arm_logic(uint32_t timestamp);
bool servo_movement(t_servo_movement *movement, uint32_t timestamp);
enum e_direction movement_direction(t_rotation_movement *config, t_servo_movement *movement);
void movement_close(t_rotation_movement *config, t_servo_movement *movement);
void movement_open(t_rotation_movement *config, t_servo_movement *movement);
void keypad_periodic(t_keypad* keypad, uint32_t timestamp);
void eeprom_write_position(uint8_t address, uint8_t position, uint8_t *prev_position);
void servo_set_position(t_servo_movement *servo, uint8_t pos);
bool timer_run(bool reset, uint32_t *timestamp, uint32_t *start, uint32_t timer);
void hand_move(enum e_direction direction);
enum e_direction hand_logic(t_keypad *keypad, t_servo_movement *hand);

/************************/
/*      FUNCTIONS       */
/************************/

bool timer_run(bool reset, uint32_t *timestamp, uint32_t *start, uint32_t timer)
{

    bool elapsed = false;
    
    if (reset == true)
    {
        /* reset the timer for a new firing */
        *start = 0;
        elapsed = true;
    }
    else if (*start == 0U)
    {
        /* start the timer */
        *start = *timestamp;
    }
    else if (*timestamp >= (*start + timer))
    {
        /* elapsed! */
        elapsed = true;
    }
 
    return elapsed;

}

void servo_set_position(t_servo_movement *servo, uint8_t pos)
{
    servo->angle_position = pos;
}

bool servo_movement(t_servo_movement *movement, uint32_t timestamp)
{

    bool movement_end = false;
    uint32_t end_time;

    if (movement->start == 0U)
    {
        movement->start = timestamp;
    }

    end_time = movement->start + movement->step_delay;

    if (movement->angle_position > movement->angle_end)
    {
        /* negative swing */
        if (movement->angle_position >= movement->angle_end)
        {
            if (timestamp >= end_time)
            {
                movement->angle_position -= movement->step_val;
                movement->start = 0;
            }
        }

        if (movement->angle_position <= movement->angle_end)
        {
            /* movement completed */
            movement_end = true;
            movement->angle_position = movement->angle_end;
            movement->start = 0;
        }
    }
    else
    {
        /* positive swing */
        if (movement->angle_position <= movement->angle_end)
        {
            if (timestamp >= end_time)
            {
                movement->angle_position += movement->step_val;
                movement->start = 0;
            }
        }

        if (movement->angle_position >= movement->angle_end)
        {
            /* movement completed */
            movement_end = true;
            movement->angle_position = movement->angle_end;
            movement->start = 0;
        }
    }

    return movement_end;

}

void arm_init(t_arm_config *config, Servo swing_servo, Servo rotation_servo, t_servo_movement *arm_swing, t_servo_movement *arm_rotation, t_servo_movement *hand)
{

    /* hardware */
    swing_servo.attach(9);
    rotation_servo.attach(8);

    /* positions */
    config->swing.angle_closed = 18;
    config->swing.angle_open = 105;
    config->rotation.angle_closed = 142;
    config->rotation.angle_open = 58;
    config->hand.angle_closed = 145;  /* this motor is NOT a Servo; thus the angles are just used to exploit the logic */
    config->hand.angle_open = 40;      /* this motor is NOT a Servo; thus the angles are just used to exploit the logic */

    /* timings */
    config->swing.angle_step = 2;
    config->swing.angle_step_delay = 55000;
    config->rotation.angle_step = 1;
    config->rotation.angle_step_delay = 28000;
    config->hand.angle_step = 1;            /* NOTE: since the motor has no feedback, keep short steps with a short delay; increase resolution (angle_open/angle_close) if needed */
    config->hand.angle_step_delay = 50500;  /* NOTE: since the motor has no feedback, keep short steps with a short delay; increase resolution (angle_open/angle_close) if needed */

    /* configure the servo movement routine */
    arm_swing->step_val = config->swing.angle_step;
    arm_swing->step_delay = config->swing.angle_step_delay;
    arm_swing->start = 0;
    arm_swing->angle_position = config->swing.angle_closed;
    arm_swing->angle_end = config->swing.angle_closed;

    arm_rotation->step_val = config->rotation.angle_step;
    arm_rotation->step_delay = config->rotation.angle_step_delay;
    arm_rotation->start = 0;
    arm_rotation->angle_position = config->rotation.angle_closed;
    arm_rotation->angle_end = config->rotation.angle_closed;

    hand->step_val = config->hand.angle_step;
    hand->step_delay = config->hand.angle_step_delay;
    hand->start = 0;
    hand->angle_position = config->hand.angle_closed;
    hand->angle_end = config->hand.angle_closed;

}

void movement_close(t_rotation_movement *config, t_servo_movement *movement)
{
    movement->angle_end = config->angle_closed;
}

void movement_open(t_rotation_movement *config, t_servo_movement *movement)
{
    movement->angle_end = config->angle_open;
}

enum e_direction movement_direction(t_rotation_movement *config, t_servo_movement *movement)
{

    if ((movement->angle_end <= config->angle_open) &&
        (movement->angle_position != movement->angle_end))
    {
        return MOV_OPEN;
    }
    else if ((movement->angle_end >= config->angle_closed) &&
             (movement->angle_position != movement->angle_end))
    {
        return MOV_CLOSE;
    }
    else
    {
        return MOV_IDLE;
    }

}

void arm_logic(uint32_t timestamp)
{

    bool elapsed;
    bool swing_movement_done = false;
    bool rot_movement_done = false;
    bool hand_movement_done = false;
    bool first_cycle = false;
    static e_arm_state state = ARM_STARTUP;
    static e_arm_state old_state = ARM_NONE;
    static uint8_t prev_pos_swing = 0;
    static uint8_t prev_pos_rot = 0;
    static uint8_t prev_pos_hand = 0;
    static uint32_t timer_start = 0;
    static uint32_t timer = 0;

    /* general purpouse timer */
    elapsed = timer_run(timer == 0 ? true : false, &timestamp, &timer_start, timer);
    if (elapsed == false) return;
    else timer = 0;

    /* ramp up-down the movement */
    swing_movement_done = servo_movement(&arm_swing, timestamp);
    rot_movement_done = servo_movement(&arm_rotation, timestamp);
    hand_movement_done = servo_movement(&hand, timestamp);

    /* write position history when neeeded */
    if (state != ARM_STARTUP)
    {
        eeprom_write_position(0, arm_swing.angle_position, &prev_pos_swing);
        eeprom_write_position(1, arm_rotation.angle_position, &prev_pos_rot);
        eeprom_write_position(2, hand.angle_position, &prev_pos_hand);
    }
    
    if (old_state != state)
    {
        swing_movement_done = false;
        rot_movement_done = false;
        hand_movement_done = false;
        first_cycle = true;
    }

    old_state = state;

    switch(state)
    {

        case ARM_STARTUP:

            /* last position is saved in the EEPROM */
            if (first_cycle == true)
            {
                /* read from eeprom */
                eeprom_read_position(0, &prev_pos_swing);
                eeprom_read_position(1, &prev_pos_rot);
                eeprom_read_position(2, &prev_pos_hand);
                /* set servo position from the eeprom */
                servo_set_position(&arm_swing, prev_pos_swing);
                servo_set_position(&arm_rotation, prev_pos_rot);
                servo_set_position(&hand, prev_pos_hand);
                /* open the arm */
                movement_open(&arm_config.swing, &arm_swing);
                /* rotate the arm */
                movement_open(&arm_config.rotation, &arm_rotation);
                /* open the hand */
                movement_open(&arm_config.hand, &hand);
            }

            if ((swing_movement_done == true) && (rot_movement_done == true) && (hand_movement_done == true))
            {
                /* boot phase completed */
                state = ARM_WAIT_BUTTON;
            }

            break;
      
        case ARM_WAIT_BUTTON:
            /* Wait for the button */
            if (keypad.buttons[BUTTON_START] == true)
            {
                /* Start pressed */
                state = ARM_HAND_OPEN;
                timer = 500000;
            }
            break;
        case ARM_HAND_OPEN:
            /* Open the hand */
            arm_config.hand.angle_open = 0;
            movement_open(&arm_config.hand, &hand);
            if (hand_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_ROTATE;
                timer = 500000;
            }
            break;
        case ARM_ROTATE:
            /* Arm rotate */
            movement_close(&arm_config.rotation, &arm_rotation);
            if (rot_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_CLOSE;
                timer = 500000;
            }
            break;
        case ARM_CLOSE:
            /* Close the arm */
            movement_close(&arm_config.swing, &arm_swing);
            if (swing_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_HAND_CLOSE;
                timer = 500000;
            }
            break;
        case ARM_HAND_CLOSE:
            /* Close the hand */
            movement_close(&arm_config.hand, &hand);
            if (hand_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_ROTATE_BACK;
                timer = 800000;
            }
            break;
        case ARM_ROTATE_BACK:
            movement_open(&arm_config.rotation, &arm_rotation);
            if (rot_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_OPEN;
                timer = 500000;
            }
            break;
        case ARM_OPEN:
            /* Reopen the arm swing */
            movement_open(&arm_config.swing, &arm_swing);
            if (swing_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_HAND_FINAL_OPEN;
            }
            break;
        case ARM_HAND_FINAL_OPEN:
            /* Open the hand */
            arm_config.hand.angle_open = 60;
            movement_open(&arm_config.hand, &hand);
            if (hand_movement_done == true)
            {
                /* movement completed, go to next state */
                state = ARM_WAIT_BUTTON;
                timer = 800000;
            }
            break;
        default:
            break;

    }

}

/* Read the keypad, apply debounce to inputs and detect the rising edge */
void keypad_periodic(t_keypad* keypad, uint32_t timestamp)
{

  uint8_t i = 0;
  bool t = false;

  for (i = 0; i < NUM_BUTTONS; i++)
  {
      t = keypad->input[i];
      
      if (t == true)
      {
          t = false;

          /* debounce the raw input */
          if (keypad->debounce[i] == 0)
              keypad->debounce[i] = timestamp;
          else
              if ((timestamp - keypad->debounce[i]) > DEBOUNCE_BUTTONS)
                  t = true;
      }
      else
      {
          keypad->debounce[i] = 0;
          t = false;
      }
      
      if ((t == true) && (keypad->latches[i] == false))
      {
          /* Falling edge */
          keypad->buttons[i] = true;
      }
      else
      {
          keypad->buttons[i] = false;
      }

      keypad->latches[i] = t;
  }

}

void eeprom_read_position(uint8_t address, uint8_t *prev_position)
{
    *prev_position = EEPROM.read(address);
    if (*prev_position > 180)
    {
        *prev_position = 0;
    }
}

void eeprom_write_position(uint8_t address, uint8_t position, uint8_t *prev_position)
{

    /* Frequently using the EEPROM is not very wise, so at least add
     * some "intelligence" to the process... */

    int16_t diff = position - *prev_position;
    diff = abs(diff);

    if (diff > 5U)
    {
        /* Movement done, write position */
        EEPROM.write(address, position);
        *prev_position = position;
    }
    else
    {
        /* Do not write the position yet */
    }
}

void setup()
{
    /* init structures */
    memset(&keypad, 0, sizeof(t_keypad));

    /* ARM hardware and software initialization */
    arm_init(&arm_config, arm_swing_servo, arm_rotation_servo, &arm_swing, &arm_rotation, &hand);

    /* I/O init */
#ifndef USE_IO_EXPANDER_BUTTONS
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);    /* start/stop button */
#endif

    /* debug serial port */
    Serial.begin(9600);

    ioExpander.begin();
    ioExpander.pinMode(0, OUTPUT); // H Bridge
    ioExpander.pinMode(1, OUTPUT); // H Bridge
    //ioExpander.pinMode(7, OUTPUT); // Debug LED xD
#ifdef USE_IO_EXPANDER_BUTTONS
    ioExpander.pinMode(BUTTON_START_PIN + BUTTON_START, INPUT_PULLUP); // Button S1 -> swing arm
    ioExpander.pinMode(BUTTON_START_PIN + BUTTON_HAND_OPEN, INPUT_PULLUP); // Button S2
    ioExpander.pinMode(BUTTON_START_PIN + BUTTON_HAND_CLOSE, INPUT_PULLUP); // Button S3
#endif

}

void hand_move(enum e_direction direction)
{
    switch(direction)
    {
        case MOV_OPEN:
      	    ioExpander.digitalWrite(0, HIGH);
      	    ioExpander.digitalWrite(1, LOW);
            break;
        case MOV_CLOSE:
      	    ioExpander.digitalWrite(0, LOW);
      	    ioExpander.digitalWrite(1, HIGH);
            break;
        case MOV_IDLE:  /* just make it explicit for easier reading */
        default:
      	    ioExpander.digitalWrite(0, LOW);
      	    ioExpander.digitalWrite(1, LOW);
            break;
    }
}

enum e_direction hand_logic(t_keypad *keypad, t_servo_movement *hand)
{

    enum e_direction dir = MOV_IDLE;

    /* otherwise movement logic is used */
    dir = movement_direction(&arm_config.hand, hand);
    /* Keypad overrides electronic intelligence :-) */
#ifdef USE_IO_EXPANDER_BUTTONS
    if (keypad->input[BUTTON_HAND_OPEN] == true)
    {
        dir = MOV_OPEN;
    }
    else if (keypad->input[BUTTON_HAND_CLOSE] == true)
    {
        dir = MOV_CLOSE;
    }
#endif
    return dir;

}

void loop()
{

    uint32_t ts = micros();
    enum e_direction dir = MOV_IDLE;

    /* Input processing */
#ifdef USE_IO_EXPANDER_BUTTONS
    keypad.input[BUTTON_START] = ioExpander.digitalRead(BUTTON_START_PIN + BUTTON_START) ? false : true;
    keypad.input[BUTTON_HAND_OPEN] = ioExpander.digitalRead(BUTTON_START_PIN + BUTTON_HAND_OPEN) ? false : true;
    keypad.input[BUTTON_HAND_CLOSE] = ioExpander.digitalRead(BUTTON_START_PIN + BUTTON_HAND_CLOSE) ? false : true;
#else
    keypad.input[BUTTON_START] = digitalRead(BUTTON_START_PIN) ? false : true;
#endif

    /* execute the program logic */
    keypad_periodic(&keypad, ts);      /* keypad */
    arm_logic(ts);                     /* arm logic and its statemachine */
    dir = hand_logic(&keypad, &hand);  /* hand logic */

    /* output processing */
    hand_move(dir);                                             /* operate the DC motor */
    arm_swing_servo.write(arm_swing.angle_position);            /* operate the servo */
    arm_rotation_servo.write(arm_rotation.angle_position);      /* operate the servo */

}


