use keyberon::action;
use keyberon::action::Action;
use keyberon::action::{k, l, HoldTapConfig};
use keyberon::key_code::KeyCode;
use keyberon::key_code::KeyCode::*;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Bootload,
    Reset,
}
const BOOTLOAD: Action<CustomActions> = Action::Custom(CustomActions::Bootload);
const RESET: Action<CustomActions> = Action::Custom(CustomActions::Reset);

const A_LS: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LShift),
    tap: &k(A),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L5_S: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(5),
    tap: &k(S),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const D_LA: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LAlt),
    tap: &k(D),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L2_F: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(2),
    tap: &k(F),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const DT_R: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RAlt),
    tap: &k(Dot),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const X_LA: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LAlt),
    tap: &k(X),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const SL_R: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RCtrl),
    tap: &k(Slash),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const Z_LC: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LCtrl),
    tap: &k(Z),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L4_C: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(4),
    tap: &k(C),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const SM_R: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RShift),
    tap: &k(SColon),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L7_S: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(7),
    tap: &k(Space),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L4_O: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(4),
    tap: &k(Comma),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};

#[allow(dead_code)]
const CT_T: Action<CustomActions> = action::m(&[KeyCode::LCtrl, KeyCode::Tab]);
#[allow(dead_code)]
const SC_T: Action<CustomActions> = action::m(&[KeyCode::LShift, KeyCode::LCtrl, KeyCode::Tab]);
#[allow(dead_code)]
const CA_D: Action<CustomActions> = action::m(&[LCtrl, LAlt, Delete]);
#[allow(dead_code)]
const AL_T: Action<CustomActions> = action::m(&[LAlt, Tab]);

pub static LAYERS: keyberon::layout::Layers<CustomActions> = keyberon::layout::layout! {
    { // 0
        [Tab    Q      W      E          R      T      Y      U      I          O      P      Enter]
        [LShift {A_LS} {L5_S} {D_LA}     {L2_F} G      H      J      K          L      {SM_R} RShift]
        [LAlt   {Z_LC} {X_LA} {L4_C}     V      B      N      M      {L4_O}     {DT_R} {SL_R} RCtrl]
        [t      t      t      {BOOTLOAD} {CT_T} BSpace {L7_S} {AL_T} {BOOTLOAD} t      t      t ]
    }
    { // 1
        [ t t t t t t t t t t t t ]
        [ t t t t t t t t t t t t ]
        [ t t t t t t t t t t t t ]
        [ t t t t t t t t t t t t ]
    }
    { // 2
        [ t t t t t t * 7 8 9 + t ]
        [ t t t t t t / 4 5 6 - t ]
        [ t t t t t t t 1 2 3 . t ]
        [ t t t t t t 0 t t t t t ]
    }
    { // 3
        [ t * 7 8 9 + t t t t t t ]
        [ t / 4 5 6 - t t t t t t ]
        [ t t 1 2 3 . t t t t t t ]
        [ t t t t t 0 t t t t t t ]
    }
    { // 4
        [ t !   @   #   $   %       t       '_' |    =   +     t   ]
        [ t '{' '}' '(' ')' t       '`'     ~   /    '"' Quote t   ]
        [ t '[' ']' ^   &   *       t       -   '\\' t   t     t   ]
        [ t t   t   t   t   {RESET} {RESET} t   t    t   t     t   ]
    }
    { // 5
        [ t t t t t t t    t    PgUp t     t     t ]
        [ t t t t t t Left Down Up   Right Enter t ]
        [ t t t t t t t    Home Down End   t     t ]
        [ t t t t t t t    t    t    t     t     t ]
    }
    { // 6
        [ t {RESET} t t t t t F7 F8 F9 MediaSleep t ]
        [ t t       t t t t t F4 F5 F6 t          t ]
        [ t t       t t t t t F1 F2 F3 t          t ]
        [ t t       t t t t t t  t  t  t          t ]
    }
    { // 7
        [ t t t t t t      MediaNextSong MediaPlayPause MediaVolDown MediaVolUp PScreen t ]
        [ t t t t t t      t             Escape         Tab          Enter      t       t ]
        [ t t t t t t      t             Home           PgDown       PgUp       End     t ]
        [ t t t t t Delete t             t              t            t          t       t ]
    }
};
