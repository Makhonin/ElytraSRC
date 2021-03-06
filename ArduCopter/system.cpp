// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Copter::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                MENU_FUNC(process_logs)},
    {"setup",               MENU_FUNC(setup_mode)},
    {"test",                MENU_FUNC(test_mode)},
    {"reboot",              MENU_FUNC(reboot_board)},
    {"help",                MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Copter::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Copter::run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void mavlink_delay_cb_static()
{
    copter.mavlink_delay_cb();
}


static void failsafe_check_static()
{
    copter.failsafe_check();
}

void Copter::init_ardupilot()
{
	// There we go!
	
	//AEROXO CODE
	
	
    if (!hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // initialise serial port
    serial_manager.init_console();

    cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();// ok
	cliSerial->print_P(PSTR("Loading BoardConfig...\n"));
    BoardConfig.init();
	cliSerial->print_P(PSTR("Loading Serial Manager...\n"));
    // initialise serial port
    serial_manager.init();
	cliSerial->print_P(PSTR("Initializing EPM...\n"));
    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif
	cliSerial->print_P(PSTR("Enabling Notify system...\n"));
    // initialise notify system
    // disable external leds if epm is enabled because of pin conflict on the APM
    notify.init(true);
	cliSerial->print_P(PSTR("Starting up battery monitor...\n"));
    // initialise battery monitor
    battery.init();
    cliSerial->print_P(PSTR("Setting Analog Source...\n"));
    rssi_analog_source      = hal.analogin->channel(g.rssi_pin);
	cliSerial->print_P(PSTR("Primary Barometer init...\n"));
    barometer.init();
	cliSerial->print_P(PSTR("Adding MavLink service callback...\n"));
    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
	cliSerial->print_P(PSTR("Connecting to USB...\n"));
    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();
	cliSerial->print_P(PSTR("Setting up UART connection 1...\n"));
    // init the GCS connected to the console
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);
	cliSerial->print_P(PSTR("Setting up UART connection 2...\n"));
    // init telemetry port
    gcs[1].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);
	cliSerial->print_P(PSTR("Setting up UART connection 3...\n"));
#if MAVLINK_COMM_NUM_BUFFERS > 2
    // setup serial port for telem2
    gcs[2].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);
#endif
	cliSerial->print_P(PSTR("Setting up UART connection 4...\n"));
#if MAVLINK_COMM_NUM_BUFFERS > 3
    // setup serial port for fourth telemetry port (not used by default)
    gcs[3].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 2);
#endif
	cliSerial->print_P(PSTR("Enabling FRSky Telemetry\n"));
#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky
    frsky_telemetry.init(serial_manager);
#endif
	cliSerial->print_P(PSTR("Setting ground station sysid...\n"));
    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
	cliSerial->print_P(PSTR("Init logging...\n"));
#if LOGGING_ENABLED == ENABLED
    log_init();
#endif
	cliSerial->print_P(PSTR("Init RC IN and OUT\n"));
    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
	cliSerial->print_P(PSTR("Setting servo relay events\n"));
    // initialise which outputs Servo and Relay events can use
    ServoRelayEvents.set_channel_mask(~motors.get_motor_mask());
	cliSerial->print_P(PSTR("Setting up relay\n"));
    relay.init();
	cliSerial->print_P(PSTR("Setup the 'main loop is dead' check\n"));
    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);
	cliSerial->print_P(PSTR("Init GPS...\n"));
    // Do GPS init
    gps.init(&DataFlash, serial_manager);
	cliSerial->print_P(PSTR("Init Compass...\n"));
    if(g.compass_enabled)
        init_compass();
	cliSerial->print_P(PSTR("Init optflow...\n"));
#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif
	cliSerial->print_P(PSTR("Init Attitude delta time...\n"));
    // initialise attitude and position controllers
    attitude_control.set_dt(MAIN_LOOP_SECONDS);
    cliSerial->print_P(PSTR("Another delta time\n"));
    pos_control.set_dt(MAIN_LOOP_SECONDS);
	cliSerial->print_P(PSTR("Init optical flow\n"));
    // init the optical flow sensor
    init_optflow();
	cliSerial->print_P(PSTR("Init camera mount\n"));
#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(serial_manager);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif
cliSerial->print_P(PSTR("IF CLI ENABLED START THERE\n"));
#if CLI_ENABLED == ENABLED
    if (g.cli_enabled) {
        const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
        cliSerial->println_P(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println_P(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println_P(msg);
        }
    }
#endif // CLI_ENABLED
cliSerial->print_P(PSTR("IF HIL MODE\n"));
#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif
cliSerial->print_P(PSTR("Init hardware baro...\n"));
    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);
cliSerial->print_P(PSTR("Init hardware sonar...\n"));
    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif
cliSerial->print_P(PSTR("Init mission library\n"));

    // initialise mission library
    mission.init();
cliSerial->print_P(PSTR("Init reset control switch...\n"));
    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();
cliSerial->print_P(PSTR("Heli init\n"));
#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
#endif
cliSerial->print_P(PSTR("Ground startup.\n"));
    startup_ground(true);
cliSerial->print_P(PSTR("Ok35\n"));
    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    cliSerial->print_P(PSTR("Ok36\n"));
    serial_manager.set_blocking_writes_all(false);
cliSerial->print_P(PSTR("Ok37\n"));
    // enable CPU failsafe
    failsafe_enable();
cliSerial->print_P(PSTR("Ok38\n"));
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);
cliSerial->print_P(PSTR("Ok39\n"));

cliSerial->print_P(PSTR("Scanning XML Elytra config...\n"));
elCfg.scanSetupFile();
attitude_control.loadAeroxoTiltrotorParameters(elCfg);

if (elCfg.getOkLoad())
{
	 gcs_send_text_P(SEVERITY_LOW,PSTR("Elytra: loaded parameters from XML!"));
}
else
{
	gcs_send_text_P(SEVERITY_LOW,PSTR("Elytra: loaded default parameters!"));
}

    cliSerial->print_P(PSTR("\nReady to FLY "));

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_ground(bool force_gyro_cal)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(force_gyro_cal?AP_InertialSensor::COLD_START:AP_InertialSensor::WARM_START,
             ins_sample_rate);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // reset ahrs gyro bias
    if (force_gyro_cal) {
        ahrs.reset_gyro_drift();
    }

    // set landed flag
    set_land_complete(true);
    set_land_complete_maybe(true);
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return ekf_position_ok();
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Copter::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Copter::optflow_position_ok()
{
#if OPTFLOW != ENABLED
    return false;
#else
    // return immediately if optflow is not enabled or EKF not used
    if (!optflow.enabled() || !ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    return (filt_status.flags.horiz_pos_rel || filt_status.flags.pred_horiz_pos_rel);
#endif
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
#if FRAME_CONFIG == HELI_FRAME 
        // if helicopters are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && !motors.rotor_runup_complete()) {
            set_auto_armed(false);
        }
#endif // HELI_FRAME
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && !ap.throttle_zero && motors.rotor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && !ap.throttle_zero) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

void Copter::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;
}

// frsky_telemetry_send - sends telemetry data using frsky telemetry
//  should be called at 5Hz by scheduler
#if FRSKY_TELEM_ENABLED == ENABLED
void Copter::frsky_telemetry_send(void)
{
    frsky_telemetry.send_frames((uint8_t)control_mode);
}
#endif

/*
  should we log a message type now?
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = motors.armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}
