/* Arduino sketch for automating powered wing mirror folding in a Nisssan
 * Qashqai
 *
 * Board design at
 * https://www.digikey.com/schemeit/project/folding-mirrors-nissan-4-D5B9OKG302JG/
 *
 * BSD licensed
 */

/* Shut our own power after 15s of inactivity (when ACC is off) */
#define POWER_TIMEOUT        15000

/* Run the mirror fold/unfold motors for this long every time
 * to account for the potential full revolution needed after
 * manual mirror folding/unfolding.
 */
#define MIRROR_POWER_TIMEOUT 8000

/* Whether to play a chime with the mirror motors on ACC on */
// #define MIRROR_MOTOR_TONES

// #define MIRROR_SWITCH_INVERT

/* Pin mapping */
#define PIN_SPEAKER          10
#define PIN_HBRIDGE_ENA      11
#define PIN_HBRIDGE_DIR1     12
#define PIN_HBRIDGE_DIR2     13
#define PIN_POWER            A0
#define PIN_MIRROR_SWITCH_IN A1
#define PIN_ACC_IN           A2
#define PIN_LOCK1_IN         A3
#define PIN_CURRENT_SENSE    A4
#define PIN_LOCK2_IN         A5

/* Actions */
enum action_e {
  ACTION_OFF,
  ACTION_WAIT,
  ACTION_MIRRORS_FOLD_IN,
  ACTION_MIRRORS_FOLD_OUT,
  ACTION_MIRRORS_STOP,
  ACTION_NOTE_C1,
  ACTION_NOTE_CS1,
  ACTION_NOTE_D1,
  ACTION_NOTE_DS1,
  ACTION_NOTE_E1,
  ACTION_NOTE_F1,
  ACTION_NOTE_FS1,
  ACTION_NOTE_G1,
  ACTION_NOTE_GS1,
  ACTION_NOTE_A1,
  ACTION_NOTE_AS1,
  ACTION_NOTE_B1,
  ACTION_NOTE_C2,
  ACTION_NOTE_OUTPUT,
  ACTION_END,
};

/* TODO: use progmem? */

static const uint16_t sequence_fold_in[] = {
  ACTION_MIRRORS_STOP,
  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
  ACTION_NOTE_B1, 100,
  ACTION_WAIT, 200,
  ACTION_NOTE_B1, 100,
  ACTION_WAIT, 50,
  ACTION_NOTE_C1, 300,
  ACTION_WAIT, 1000,
  ACTION_MIRRORS_FOLD_IN,
  ACTION_WAIT, MIRROR_POWER_TIMEOUT,
  ACTION_MIRRORS_STOP,
  ACTION_END
};

static const uint16_t sequence_fold_out[] = {
  ACTION_MIRRORS_STOP,
  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
  ACTION_NOTE_C1, 100,
  ACTION_WAIT, 200,
  ACTION_NOTE_C1, 100,
  ACTION_WAIT, 50,
  ACTION_NOTE_B1, 300,
  ACTION_WAIT, 1000,
  ACTION_MIRRORS_FOLD_OUT,
  ACTION_WAIT, MIRROR_POWER_TIMEOUT,
  ACTION_MIRRORS_STOP,
  ACTION_END
};

static const uint16_t sequence_off[] = {
//  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
//  ACTION_NOTE_DS1, 100,
//  ACTION_WAIT, 50,
//  ACTION_NOTE_C1, 100,
  ACTION_OFF,
  ACTION_END
};

#define ACC_ON_CHIME   \
  ACTION_NOTE_A1, 400, \
  ACTION_WAIT, 100,    \
  ACTION_NOTE_A1, 400, \
  ACTION_WAIT, 100,    \
  ACTION_NOTE_A1, 400, \
  ACTION_WAIT, 100,    \
  ACTION_NOTE_F1, 250, \
  ACTION_WAIT, 75,     \
  ACTION_NOTE_C2, 125, \
  ACTION_NOTE_A1, 400, \
  ACTION_WAIT, 100,    \
  ACTION_NOTE_F1, 250, \
  ACTION_WAIT, 75,     \
  ACTION_NOTE_C2, 125, \
  ACTION_NOTE_A1, 400

static const uint16_t sequence_acc_on[] = {
  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
  ACC_ON_CHIME,
  ACTION_END
};

static const uint16_t sequence_acc_on_fold_out[] = {
#ifdef MIRROR_MOTOR_TONES
  ACTION_MIRRORS_FOLD_IN,
  /* Make sure the mirror is not hitting the endstop */
  ACTION_WAIT, 200,
  /* Even though not a PWM pin, modulating the DIR1 pin
   * makes for a louder sound than modulating ENA because
   * the motor alternates between running and braking
   * rather than running and freewheeling.  Reversing the
   * motor would probably work even better.  TODO:
   * also play with the PWM duty cycle.
   */
  ACTION_NOTE_OUTPUT, PIN_HBRIDGE_DIR1,
#else
  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
#endif
  ACC_ON_CHIME,
  ACTION_MIRRORS_STOP,
  ACTION_WAIT, 200,
  ACTION_MIRRORS_FOLD_OUT,
  ACTION_WAIT, MIRROR_POWER_TIMEOUT,
  ACTION_MIRRORS_STOP,
  ACTION_END
};

static const uint16_t sequence_beep[] = {
  ACTION_NOTE_OUTPUT, PIN_SPEAKER,
  ACTION_NOTE_F1, 200,
  ACTION_END,
};

static void mirrors_open(void) {
  digitalWrite(PIN_HBRIDGE_DIR2, LOW);
  digitalWrite(PIN_HBRIDGE_DIR1, HIGH);
  digitalWrite(PIN_HBRIDGE_ENA, HIGH);
}

static void mirrors_fold(void) {
  digitalWrite(PIN_HBRIDGE_DIR1, LOW);
  digitalWrite(PIN_HBRIDGE_DIR2, HIGH);
  digitalWrite(PIN_HBRIDGE_ENA, HIGH);
}

static void mirrors_stop(void) {
  digitalWrite(PIN_HBRIDGE_ENA, LOW);
}

static void off(void) {
  mirrors_stop();
  digitalWrite(PIN_POWER, LOW);
}

static const uint16_t action_to_freq[] = {
  /* Non-trivial designated intializers unimplemented in the
   * gcc version some Arduino versions ship.
   */
  /*[ACTION_NOTE_C1]  = */523,
  /*[ACTION_NOTE_CS1] = */554,
  /*[ACTION_NOTE_D1]  = */587,
  /*[ACTION_NOTE_DS1] = */622,
  /*[ACTION_NOTE_E1]  = */659,
  /*[ACTION_NOTE_F1]  = */698,
  /*[ACTION_NOTE_FS1] = */740,
  /*[ACTION_NOTE_G1]  = */784,
  /*[ACTION_NOTE_GS1] = */832,
  /*[ACTION_NOTE_A1]  = */880,
  /*[ACTION_NOTE_AS1] = */932,
  /*[ACTION_NOTE_B1]  = */988,
  /*[ACTION_NOTE_C2]  = */1047,
};

static const uint16_t *seq_step;
static uint16_t seq_next_ts;
static uint8_t seq_note_pin;

static void sequence_set(const uint16_t *seq) {
  seq_step = seq;
  seq_next_ts = millis();
}

static void sequence_run(void) {
  while (seq_step) {
    uint16_t action;
    uint16_t freq;
    uint16_t now = millis();

    if ((int16_t) (seq_next_ts - now) > 0)
      break;

    action = *seq_step++;
    switch (action) {
    case ACTION_OFF:
      off();
      break;
    case ACTION_WAIT:
      seq_next_ts += *seq_step++;
      break;
    case ACTION_MIRRORS_FOLD_IN:
      mirrors_fold();
      break;
    case ACTION_MIRRORS_FOLD_OUT:
      mirrors_open();
      break;
    case ACTION_MIRRORS_STOP:
      mirrors_stop();
      break;
    case ACTION_NOTE_C1 ... ACTION_NOTE_C2:
      freq = action_to_freq[action - ACTION_NOTE_C1];
      if (seq_note_pin != PIN_SPEAKER)
        freq >>= 3;
      tone(seq_note_pin, freq, *seq_step);
      seq_next_ts += *seq_step++;
      break;
    case ACTION_NOTE_OUTPUT:
      seq_note_pin = *seq_step++;
      break;
    case ACTION_END:
      seq_step = NULL;
      return;
    }

    while (*seq_step == ACTION_WAIT) {
      seq_next_ts += seq_step[1];
      seq_step += 2;
    }

    if (*seq_step == ACTION_END)
      seq_step = NULL;
  }
}

static bool acc_sense(void) {
  /* No special handling here, the pin is pulled down */
  return digitalRead(PIN_ACC_IN);
}

static int lock_sense(void) {
  /* TODO: this probably needs more work, debouncing, etc. */
  /* TODO: could also use just one pin, read it as analog over
   * some period of time and deduce whether we're locking or
   * unlocking.
   * On lock the lock positive pin will see close to 12V maximum,
   * on unlock still a non-zero maximum (the multimeter shows
   * around 3V) and on idle it'll stay at 0 if pulled down,
   * which it is.
   */
  if (digitalRead(PIN_LOCK1_IN))
    return 1;
  if (digitalRead(PIN_LOCK2_IN))
    return -1;
  return 0;
}

static int switch_sense(void) {
  /* Pull down mode to check for FOLD switch position */
  /* Do we need to debounce? */
  pinMode(PIN_MIRROR_SWITCH_IN, INPUT);
  delay(10);
  if (digitalRead(PIN_MIRROR_SWITCH_IN))
    return -1;
  /* Pull up mode to check for OPEN switch position */
  pinMode(PIN_MIRROR_SWITCH_IN, INPUT_PULLUP);
  delay(10);
  if (!digitalRead(PIN_MIRROR_SWITCH_IN))
    return 1;

  /* If it's neither high or low, it's Hi-Z, i.e. in the middle position  */
  return 0;
}

static uint16_t activity_ts = 0;
static uint16_t acc_off_ts = 0;
static int lock, acc, sw;
/* Whether the car is locked as opposed to whether it is locking/unlocking
 * right now.  By default 0 (undefined).
 */
static int locked;

void setup(void) {
  /*
   * The important part first: the power pin level decides if
   * the arduino gets any power after the LOCK and UNLOCK pulses
   * end and the voltage on these pins goes back to low so if we
   * take too long to write the POWER pin HIGH we may not reach
   * the rest of the code here.
   */
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);

  pinMode(PIN_HBRIDGE_DIR1, OUTPUT);
  pinMode(PIN_HBRIDGE_DIR2, OUTPUT);
  pinMode(PIN_HBRIDGE_ENA, OUTPUT);
  mirrors_stop();

  /* With this out of the way set up the inputs etc. */
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_MIRROR_SWITCH_IN, INPUT);
  pinMode(PIN_ACC_IN, INPUT);
  pinMode(PIN_LOCK1_IN, INPUT);
  pinMode(PIN_LOCK2_IN, INPUT);
  pinMode(PIN_CURRENT_SENSE, INPUT);

  /* The folding and unfolding of the mirrors is based on three
   * inputs, the lock/unlocking of the door, the ACC rail state,
   * and the manual mirror switch state.  In the case of
   * lock/unlock/ACC, when we wake up (arduino first receives
   * power from either source) we will act immediately on the
   * value we read from these inputs because the voltage
   * change on those wires is probably what has cause us to wake
   * up.  If there had been no change we wouldn't have awoken.
   * So we left inital values for lock and acc as 0, and if
   * setup() reads them as 1, it'll notice a change in the value
   * and act on it.
   *
   * For the sw state we may have a policy to act on the initial
   * state or not, we have no way to know if the user had just
   * actuated the switch or it's been in the current state for a
   * while, until we are fully running and are tracking the state
   * of the switch.  For now leave it at 0 as well.
   */
}

void loop(void) {
  /* lock_sense() must be the first thing we do after setup()
   * so that we can read and react to the short pulse on the
   * lock power wire(s) which may be what has waken us up in the
   * first place.  We may have little time left before that
   * pulse ends.
   */
  int new_lock = lock_sense();
  int new_acc = acc_sense();
  int new_sw = switch_sense();
  const uint16_t *act = NULL;

  if (new_lock != lock) {
    lock = new_lock;
    if (!seq_step)
      act = sequence_beep;

    if (lock) {
      /* If lock is 1 or -1 and the value just changed we must be
       * locking or unlocking right now, try to start the
       * respective action immediately when ACC is off and switch
       * is in the auto positiion.  Otherwise  do nothing.
       */
      if (!new_acc && lock != locked && new_sw == 0)
        act = lock > 0 ? sequence_fold_in : sequence_fold_out;

      locked = lock;
    }
  }

  /* Overrides previous action */
  if (new_acc != acc) {
    acc = new_acc;

    /* Skip the action on momentary acc off such as
     * with the starter running.
     */
    if (acc && (!acc_off_ts ||
        (uint16_t) (millis() - acc_off_ts) > 2000)) {
      if (new_sw >= 0) {
        /* Unfold mirrors on ACC power on event unless user
         * explicitly wants them folded in.
         */
        act = sequence_acc_on_fold_out;
      } else
        act = sequence_acc_on;
    } else if (!acc) {
      /* Update activity_ts to keep ourselves powered for
       * POWER_TIMEOUT ms after ACC is turned off.
       */
      activity_ts = acc_off_ts = millis();

      if (!seq_step)
        act = sequence_beep;
    }
  }

  if (new_sw != sw) {
    sw = new_sw;
    if (!seq_step)
      sequence_set(sequence_beep);

    if (sw) {
      /* Act on the switch state change signal unconditionally
       * overriding all other inputs.
       */
      act = sw > 0 ? sequence_fold_out : sequence_fold_in;
    }
  }

  if (act) {
    sequence_set(act);
    activity_ts = millis();
  } else if (!acc &&
      (uint16_t) millis() - (uint16_t) activity_ts >=
      POWER_TIMEOUT)
    sequence_set(sequence_off);

  sequence_run();
}
