--[[
    FlightDataRecorder.lua  —  FlyWithLua NG for X-Plane 12
    Comprehensive flight + systems snapshot with adaptive sampling rate.
    - Normal phase  : 1 snapshot every 2 s  (0.5 Hz)
    - Critical phase: 4 snapshots per second (4 Hz)
      Trigger: radio altitude < 300 ft  AND  IAS > 40 kts
      (covers takeoff roll, climb-out, approach, landing, go-around)
    Covers cold-start sequence: electrical, APU, engine start,
    fuel, anti-ice, pressurization, hydraulics, EFIS, switches.
    Output: X-Plane 12/Output/flight_recorder/flight_YYYYMMDD_HHMMSS.jsonl
--]]

local INTERVAL_NORMAL   = 2.0   -- seconds between snapshots in normal flight
local INTERVAL_CRITICAL = 0.25  -- seconds between snapshots in critical phase (4 Hz)
local last_write_time   = 0.0   -- uses TOTAL_RUNNING_TIME_SEC (float, high-res)

local output_dir   = SYSTEM_DIRECTORY .. "Output/flight_recorder/"
os.execute('mkdir -p "' .. output_dir .. '"')
local session_file = output_dir .. "flight_" .. os.date("%Y%m%d_%H%M%S") .. ".jsonl"

local function mul(v, f) return type(v)=="number" and v*f or nil end
local function div(v, f) return type(v)=="number" and f~=0 and v/f or nil end
local function bool(v)   return v == 1 end

-- Safe dataref: registers only if the DataRef actually exists in this session.
-- Prevents quarantine when optional/aircraft-specific DataRefs are absent.
local function safe_dataref(varname, path, access, idx)
    local ok = pcall(function()
        if idx then dataref(varname, path, access, idx)
        else        dataref(varname, path, access) end
    end)
    if not ok then print("[FDR] optional DataRef not found: " .. path) end
end

-- ================================================================
-- DataRef declarations  (per-index for all arrays)
-- ================================================================

-- ---- Position ----
dataref("fdr_lat",       "sim/flightmodel/position/latitude",                              "readonly")
dataref("fdr_lon",       "sim/flightmodel/position/longitude",                             "readonly")
dataref("fdr_alt_msl_m", "sim/flightmodel/position/elevation",                             "readonly")
dataref("fdr_alt_agl_m", "sim/flightmodel/position/y_agl",                                 "readonly")
dataref("fdr_ra_ft",     "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot",  "readonly")
dataref("fdr_alt_ind",   "sim/cockpit2/gauges/indicators/altitude_ft_pilot",               "readonly")
dataref("fdr_mag_var",   "sim/flightmodel/position/magnetic_variation",                    "readonly")

-- ---- Attitude ----
dataref("fdr_pitch",     "sim/flightmodel/position/true_theta",   "readonly")
dataref("fdr_roll",      "sim/flightmodel/position/true_phi",     "readonly")
dataref("fdr_true_hdg",  "sim/flightmodel/position/true_psi",     "readonly")
dataref("fdr_mag_hdg",   "sim/flightmodel/position/mag_psi",      "readonly")

-- ---- Speed & Flight Path ----
dataref("fdr_ias",       "sim/flightmodel/position/indicated_airspeed",                        "readonly")
dataref("fdr_ias_ind",   "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",                  "readonly")
dataref("fdr_ias_trend", "sim/cockpit2/gauges/indicators/airspeed_acceleration_kts_sec_pilot", "readonly")
dataref("fdr_tas_ms",    "sim/flightmodel/position/true_airspeed",                             "readonly")
dataref("fdr_gs_ms",     "sim/flightmodel/position/groundspeed",                               "readonly")
dataref("fdr_vs_fpm",    "sim/flightmodel/position/vh_ind_fpm",                                "readonly")
dataref("fdr_vvi_ind",   "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",                       "readonly")
dataref("fdr_mach",      "sim/flightmodel/misc/machno",                                        "readonly")
dataref("fdr_vpath",     "sim/flightmodel/position/vpath",                                     "readonly")
dataref("fdr_hpath",     "sim/flightmodel/position/hpath",                                     "readonly")

-- ---- G-forces ----
dataref("fdr_g_normal",  "sim/flightmodel/forces/g_nrml",  "readonly")
dataref("fdr_g_axial",   "sim/flightmodel/forces/g_axil",  "readonly")
dataref("fdr_g_lat",     "sim/flightmodel/forces/g_side",  "readonly")

-- ---- Aerodynamics ----
dataref("fdr_aoa",       "sim/flightmodel/position/alpha",       "readonly")
dataref("fdr_sideslip",  "sim/flightmodel/position/beta",        "readonly")
dataref("fdr_roll_rate", "sim/flightmodel/position/P",           "readonly")
dataref("fdr_pitch_rate","sim/flightmodel/position/Q",           "readonly")
dataref("fdr_yaw_rate",  "sim/flightmodel/position/R",           "readonly")
dataref("fdr_cl",        "sim/flightmodel/misc/cl_overall",      "readonly")
dataref("fdr_cd",        "sim/flightmodel/misc/cd_overall",      "readonly")
dataref("fdr_turnrate",  "sim/flightmodel/misc/turnrate_noroll", "readonly")
dataref("fdr_slip",      "sim/flightmodel/misc/slip",            "readonly")
dataref("fdr_lift_n",    "sim/flightmodel/forces/fnrml_aero",    "readonly")
dataref("fdr_drag_n",    "sim/flightmodel/forces/faxil_aero",    "readonly")

-- ---- Pilot Controls ----
dataref("fdr_yoke_pitch",  "sim/joystick/yoke_pitch_ratio",                       "readonly")
dataref("fdr_yoke_roll",   "sim/joystick/yoke_roll_ratio",                        "readonly")
dataref("fdr_yoke_yaw",    "sim/joystick/yoke_heading_ratio",                     "readonly")
dataref("fdr_brake_l",     "sim/cockpit2/controls/left_brake_ratio",              "readonly")
dataref("fdr_brake_r",     "sim/cockpit2/controls/right_brake_ratio",             "readonly")
dataref("fdr_speedbrake",  "sim/cockpit2/controls/speedbrake_ratio",              "readonly")
dataref("fdr_sbrkrat",     "sim/flightmodel/controls/sbrkrat",                    "readonly")
dataref("fdr_flap_handle", "sim/cockpit2/controls/flap_handle_deploy_ratio",      "readonly")
dataref("fdr_flap_actual", "sim/flightmodel/controls/flaprat",                    "readonly")
dataref("fdr_slat_actual", "sim/flightmodel/controls/slatrat",                    "readonly")
dataref("fdr_gear_handle", "sim/cockpit/switches/gear_handle_status",             "readonly")
dataref("fdr_park_brake",  "sim/flightmodel/controls/parkbrake",                  "readonly")
dataref("fdr_elev_trim",   "sim/flightmodel/controls/elv_trim",                   "readonly")
dataref("fdr_ail_trim",    "sim/flightmodel/controls/ail_trim",                   "readonly")
dataref("fdr_rud_trim",    "sim/flightmodel/controls/rud_trim",                   "readonly")

-- ---- Actual Surface Deflections (deg) ----
dataref("fdr_lail_def",  "sim/flightmodel/controls/lail1def",  "readonly")
dataref("fdr_rail_def",  "sim/flightmodel/controls/rail1def",  "readonly")
dataref("fdr_lrud_def",  "sim/flightmodel/controls/ldruddef",  "readonly")
dataref("fdr_rrud_def",  "sim/flightmodel/controls/rdruddef",  "readonly")
dataref("fdr_lspl_def",  "sim/flightmodel/controls/lsplrdef",  "readonly")
dataref("fdr_rspl_def",  "sim/flightmodel/controls/rsplrdef",  "readonly")

-- ---- Landing Gear ----
dataref("fdr_gear_d0",   "sim/flightmodel2/gear/deploy_ratio",    "readonly", 0)
dataref("fdr_gear_d1",   "sim/flightmodel2/gear/deploy_ratio",    "readonly", 1)
dataref("fdr_gear_d2",   "sim/flightmodel2/gear/deploy_ratio",    "readonly", 2)
dataref("fdr_gear_g0",   "sim/flightmodel2/gear/on_ground",       "readonly", 0)
dataref("fdr_gear_g1",   "sim/flightmodel2/gear/on_ground",       "readonly", 1)
dataref("fdr_gear_g2",   "sim/flightmodel2/gear/on_ground",       "readonly", 2)
dataref("fdr_on_ground", "sim/flightmodel/failures/onground_all", "readonly")

-- ---- Engines (flight data) ----
dataref("fdr_thr0",   "sim/flightmodel2/engines/throttle_used_ratio",        "readonly", 0)
dataref("fdr_thr1",   "sim/flightmodel2/engines/throttle_used_ratio",        "readonly", 1)
dataref("fdr_thrr0",  "sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", "readonly", 0)
dataref("fdr_thrr1",  "sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", "readonly", 1)
dataref("fdr_n1_0",   "sim/flightmodel2/engines/N1_percent",                  "readonly", 0)
dataref("fdr_n1_1",   "sim/flightmodel2/engines/N1_percent",                  "readonly", 1)
dataref("fdr_n2_0",   "sim/flightmodel2/engines/N2_percent",                  "readonly", 0)
dataref("fdr_n2_1",   "sim/flightmodel2/engines/N2_percent",                  "readonly", 1)
dataref("fdr_egt0",   "sim/cockpit2/engine/indicators/EGT_deg_C",             "readonly", 0)
dataref("fdr_egt1",   "sim/cockpit2/engine/indicators/EGT_deg_C",             "readonly", 1)
dataref("fdr_itt0",   "sim/cockpit2/engine/indicators/ITT_deg_cel",           "readonly", 0)
dataref("fdr_itt1",   "sim/cockpit2/engine/indicators/ITT_deg_cel",           "readonly", 1)
dataref("fdr_epr0",   "sim/cockpit2/engine/indicators/EPR_ratio",             "readonly", 0)
dataref("fdr_epr1",   "sim/cockpit2/engine/indicators/EPR_ratio",             "readonly", 1)
dataref("fdr_ff0",    "sim/cockpit2/engine/indicators/fuel_flow_kg_sec",      "readonly", 0)
dataref("fdr_ff1",    "sim/cockpit2/engine/indicators/fuel_flow_kg_sec",      "readonly", 1)
dataref("fdr_oilp0",  "sim/cockpit2/engine/indicators/oil_pressure_psi",      "readonly", 0)
dataref("fdr_oilp1",  "sim/cockpit2/engine/indicators/oil_pressure_psi",      "readonly", 1)
dataref("fdr_oilt0",  "sim/cockpit2/engine/indicators/oil_temperature_deg_C", "readonly", 0)
dataref("fdr_oilt1",  "sim/cockpit2/engine/indicators/oil_temperature_deg_C", "readonly", 1)
dataref("fdr_thr_n0", "sim/cockpit2/engine/indicators/thrust_n",              "readonly", 0)
dataref("fdr_thr_n1", "sim/cockpit2/engine/indicators/thrust_n",              "readonly", 1)
dataref("fdr_eon0",   "sim/flightmodel2/engines/engine_is_burning_fuel",      "readonly", 0)
dataref("fdr_eon1",   "sim/flightmodel2/engines/engine_is_burning_fuel",      "readonly", 1)
dataref("fdr_efi0",   "sim/cockpit2/annunciators/engine_fires",               "readonly", 0)
dataref("fdr_efi1",   "sim/cockpit2/annunciators/engine_fires",               "readonly", 1)
dataref("fdr_rev0",   "sim/cockpit2/annunciators/reverser_on",                "readonly", 0)
dataref("fdr_rev1",   "sim/cockpit2/annunciators/reverser_on",                "readonly", 1)

-- ---- Thrust Reverser Deploy (0=stowed, 1.0=fully deployed) ----
dataref("fdr_rev_d0", "sim/flightmodel2/engines/thrust_reverser_deploy_ratio", "readonly", 0)
dataref("fdr_rev_d1", "sim/flightmodel2/engines/thrust_reverser_deploy_ratio", "readonly", 1)

-- ---- Engine Start Switches ----
dataref("fdr_eng_mode",  "sim/cockpit2/engine/actuators/eng_mode_selector",  "readonly")      -- 0=Norm,-1=Crank,1=Ign,2=Air
dataref("fdr_emast0",    "sim/cockpit2/engine/actuators/eng_master",         "readonly", 0)
dataref("fdr_emast1",    "sim/cockpit2/engine/actuators/eng_master",         "readonly", 1)
dataref("fdr_ign0",      "sim/cockpit2/engine/actuators/ignition_on",        "readonly", 0)   -- 0=off,1=L,2=R,3=both
dataref("fdr_ign1",      "sim/cockpit2/engine/actuators/ignition_on",        "readonly", 1)
dataref("fdr_ikey0",     "sim/cockpit2/engine/actuators/ignition_key",       "readonly", 0)   -- 4=starting
dataref("fdr_ikey1",     "sim/cockpit2/engine/actuators/ignition_key",       "readonly", 1)
dataref("fdr_strt0",     "sim/cockpit2/engine/actuators/starter_hit",        "readonly", 0)
dataref("fdr_strt1",     "sim/cockpit2/engine/actuators/starter_hit",        "readonly", 1)
dataref("fdr_fadec0",    "sim/cockpit2/engine/actuators/fadec_on",           "readonly", 0)
dataref("fdr_fadec1",    "sim/cockpit2/engine/actuators/fadec_on",           "readonly", 1)
dataref("fdr_epump0",    "sim/cockpit2/engine/actuators/fuel_pump_on",       "readonly", 0)   -- per-engine fuel pump
dataref("fdr_epump1",    "sim/cockpit2/engine/actuators/fuel_pump_on",       "readonly", 1)

-- ---- APU ----
dataref("fdr_apu_sw",    "sim/cockpit2/electrical/APU_starter_switch",  "readonly")  -- 0=off,1=on,2=start
dataref("fdr_apu_n1",    "sim/cockpit2/electrical/APU_N1_percent",      "readonly")
dataref("fdr_apu_egt",   "sim/cockpit2/electrical/APU_EGT_c",           "readonly")
dataref("fdr_apu_run",   "sim/cockpit2/electrical/APU_running",         "readonly")
dataref("fdr_apu_gen",   "sim/cockpit2/electrical/APU_generator_on",    "readonly")
dataref("fdr_apu_amps",  "sim/cockpit2/electrical/APU_generator_amps",  "readonly")
dataref("fdr_apu_door",  "sim/cockpit2/electrical/APU_door",            "readonly")

-- ---- Electrical System ----
dataref("fdr_bat0",      "sim/cockpit2/electrical/battery_on",                    "readonly", 0)
dataref("fdr_bat1",      "sim/cockpit2/electrical/battery_on",                    "readonly", 1)
dataref("fdr_bat_v0",    "sim/cockpit2/electrical/battery_voltage_actual_volts",  "readonly", 0)
dataref("fdr_bat_v1",    "sim/cockpit2/electrical/battery_voltage_actual_volts",  "readonly", 1)
dataref("fdr_bat_a0",    "sim/cockpit2/electrical/battery_amps",                  "readonly", 0)
dataref("fdr_bat_a1",    "sim/cockpit2/electrical/battery_amps",                  "readonly", 1)
dataref("fdr_gen0",      "sim/cockpit2/electrical/generator_on",                  "readonly", 0)
dataref("fdr_gen1",      "sim/cockpit2/electrical/generator_on",                  "readonly", 1)
dataref("fdr_gen_a0",    "sim/cockpit2/electrical/generator_amps",                "readonly", 0)
dataref("fdr_gen_a1",    "sim/cockpit2/electrical/generator_amps",                "readonly", 1)
dataref("fdr_gen_v0",    "sim/cockpit2/electrical/generator_volts",               "readonly", 0)
dataref("fdr_gen_v1",    "sim/cockpit2/electrical/generator_volts",               "readonly", 1)
dataref("fdr_gpu_gen",   "sim/cockpit2/electrical/GPU_generator_on",              "readonly")
dataref("fdr_gpu_v",     "sim/cockpit2/electrical/GPU_generator_volts",           "readonly")
dataref("fdr_gpu_avail", "sim/cockpit2/electrical/GPU_can_be_called_for",         "readonly")
dataref("fdr_avion",     "sim/cockpit2/switches/avionics_power_on",               "readonly")
dataref("fdr_cross_tie", "sim/cockpit2/electrical/cross_tie",                     "readonly")
dataref("fdr_bus_v0",    "sim/cockpit2/electrical/bus_volts",  "readonly", 0)
dataref("fdr_bus_v1",    "sim/cockpit2/electrical/bus_volts",  "readonly", 1)
dataref("fdr_bus_v2",    "sim/cockpit2/electrical/bus_volts",  "readonly", 2)
dataref("fdr_bus_v3",    "sim/cockpit2/electrical/bus_volts",  "readonly", 3)

-- ---- Fuel System ----
dataref("fdr_fuel_total",   "sim/flightmodel/weight/m_fuel_total",        "readonly")
dataref("fdr_fuel0",        "sim/flightmodel/weight/m_fuel",              "readonly", 0)
dataref("fdr_fuel1",        "sim/flightmodel/weight/m_fuel",              "readonly", 1)
dataref("fdr_fuel2",        "sim/flightmodel/weight/m_fuel",              "readonly", 2)
dataref("fdr_fuelq0",       "sim/cockpit2/fuel/fuel_quantity",            "readonly", 0)  -- indicated
dataref("fdr_fuelq1",       "sim/cockpit2/fuel/fuel_quantity",            "readonly", 1)
dataref("fdr_fuelq2",       "sim/cockpit2/fuel/fuel_quantity",            "readonly", 2)
dataref("fdr_tank_pump0",   "sim/cockpit2/fuel/fuel_tank_pump_on",        "readonly", 0)
dataref("fdr_tank_pump1",   "sim/cockpit2/fuel/fuel_tank_pump_on",        "readonly", 1)
dataref("fdr_tank_pump2",   "sim/cockpit2/fuel/fuel_tank_pump_on",        "readonly", 2)
dataref("fdr_xfeed",        "sim/cockpit2/fuel/fuel_crossfeed_selector",  "readonly")
dataref("fdr_xfeed_auto",   "sim/cockpit2/fuel/auto_crossfeed",           "readonly")
dataref("fdr_fw_l",         "sim/cockpit2/fuel/firewall_closed_left",     "readonly")
dataref("fdr_fw_r",         "sim/cockpit2/fuel/firewall_closed_right",    "readonly")
dataref("fdr_sel_l",        "sim/cockpit2/fuel/fuel_tank_selector_left",  "readonly")
dataref("fdr_sel_r",        "sim/cockpit2/fuel/fuel_tank_selector_right", "readonly")

-- ---- Anti-ice & De-ice ----
dataref("fdr_ai_eng0",    "sim/cockpit2/ice/ice_inlet_heat_on_per_engine",  "readonly", 0)
dataref("fdr_ai_eng1",    "sim/cockpit2/ice/ice_inlet_heat_on_per_engine",  "readonly", 1)
dataref("fdr_ai_wing_l",  "sim/cockpit2/ice/ice_surface_hot_bleed_air_left_on",   "readonly")
dataref("fdr_ai_wing_r",  "sim/cockpit2/ice/ice_surface_hot_bleed_air_right_on",  "readonly")
dataref("fdr_ai_pitot_p", "sim/cockpit2/ice/ice_pitot_heat_on_pilot",       "readonly")
dataref("fdr_ai_pitot_c", "sim/cockpit2/ice/ice_pitot_heat_on_copilot",     "readonly")
dataref("fdr_ai_pitot_s", "sim/cockpit2/ice/ice_pitot_heat_on_standby",     "readonly")
dataref("fdr_ai_wnd",     "sim/cockpit2/ice/ice_window_heat_on",            "readonly")
dataref("fdr_ai_aoa",     "sim/cockpit2/ice/ice_AOA_heat_on",               "readonly")
dataref("fdr_ai_stat_p",  "sim/cockpit2/ice/ice_static_heat_on_pilot",      "readonly")
dataref("fdr_ai_stat_c",  "sim/cockpit2/ice/ice_static_heat_on_copilot",    "readonly")
dataref("fdr_ai_tat",     "sim/cockpit2/ice/ice_TAT_heat_on",               "readonly")

-- ---- Pressurization / ECS ----
dataref("fdr_bleed_mode", "sim/cockpit2/pressurization/actuators/bleed_air_mode",          "readonly")  -- 0=off,1=L,2=both,3=R,4=APU,5=auto
dataref("fdr_cab_alt_cmd","sim/cockpit2/pressurization/actuators/cabin_altitude_ft",        "readonly")
dataref("fdr_cab_alt",    "sim/cockpit2/pressurization/indicators/cabin_altitude_ft",       "readonly")
dataref("fdr_cab_vvi",    "sim/cockpit2/pressurization/indicators/cabin_vvi_fpm",           "readonly")
dataref("fdr_cab_dp",     "sim/cockpit2/pressurization/indicators/pressure_diffential_psi", "readonly")
dataref("fdr_outflow_v",  "sim/cockpit2/pressurization/indicators/outflow_valve",           "readonly")
dataref("fdr_air_cond",   "sim/cockpit2/pressurization/actuators/air_cond_on",              "readonly")

-- ---- Hydraulics ----
dataref("fdr_hyd1",       "sim/cockpit2/switches/electric_hydraulic_pump_on",                     "readonly")
dataref("fdr_hyd2",       "sim/cockpit2/switches/electric_hydraulic_pump2_on",                    "readonly")
dataref("fdr_hyd_p1",     "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1",              "readonly")
dataref("fdr_hyd_p2",     "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2",              "readonly")
dataref("fdr_hyd_p3",     "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3",              "readonly")
dataref("fdr_hyd_f1",     "sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_1",           "readonly")
dataref("fdr_hyd_f2",     "sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_2",           "readonly")
dataref("fdr_hyd_brkacc", "sim/cockpit2/hydraulics/indicators/brake_accumulator_pressure_ratio",  "readonly")
dataref("fdr_ptu",        "sim/cockpit2/hydraulics/actuators/PTU",                                "readonly")  -- 0=Off,1=Armed,2=Running

-- ---- Elevator Actual Deflection (index 0=left stab, 1=right stab) ----
dataref("fdr_elev_l", "sim/flightmodel2/wing/elevator1_deg", "readonly", 0)
dataref("fdr_elev_r", "sim/flightmodel2/wing/elevator1_deg", "readonly", 1)

-- ---- Nosewheel Steering & Brake Energy ----
dataref("fdr_steer_cmd", "sim/flightmodel2/gear/tire_steer_command_deg", "readonly", 0)
dataref("fdr_steer_act", "sim/flightmodel2/gear/tire_steer_actual_deg",  "readonly", 0)
dataref("fdr_brk_J0",    "sim/flightmodel2/gear/brake_absorbed_J",       "readonly", 0)  -- nose
dataref("fdr_brk_J1",    "sim/flightmodel2/gear/brake_absorbed_J",       "readonly", 1)  -- left main
dataref("fdr_brk_J2",    "sim/flightmodel2/gear/brake_absorbed_J",       "readonly", 2)  -- right main
dataref("fdr_brk_rat0",  "sim/flightmodel2/gear/brake_absorbed_rat",     "readonly", 1)  -- left main fade
dataref("fdr_brk_rat1",  "sim/flightmodel2/gear/brake_absorbed_rat",     "readonly", 2)  -- right main fade
-- Tire skid ratio: 0=full traction, 1=completely skidding (indicates anti-skid activation)
dataref("fdr_skid0",     "sim/flightmodel2/gear/tire_skid_ratio",        "readonly", 0)  -- nose
dataref("fdr_skid1",     "sim/flightmodel2/gear/tire_skid_ratio",        "readonly", 1)  -- left main
dataref("fdr_skid2",     "sim/flightmodel2/gear/tire_skid_ratio",        "readonly", 2)  -- right main

-- ---- Autopilot ----
dataref("fdr_ap_mode",    "sim/cockpit/autopilot/autopilot_mode",              "readonly")
dataref("fdr_ap_state",   "sim/cockpit/autopilot/autopilot_state",             "readonly")
dataref("fdr_ap_lat_mode","sim/cockpit2/autopilot/heading_mode",               "readonly")
dataref("fdr_ap_vrt_mode","sim/cockpit2/autopilot/altitude_mode",              "readonly")
dataref("fdr_at_enabled", "sim/cockpit2/autopilot/autothrottle_enabled",       "readonly")
dataref("fdr_at_on",      "sim/cockpit2/autopilot/autothrottle_on",            "readonly")
dataref("fdr_ap_hdg",     "sim/cockpit/autopilot/heading_mag",                 "readonly")
dataref("fdr_ap_alt",     "sim/cockpit/autopilot/altitude",                    "readonly")
dataref("fdr_ap_spd",     "sim/cockpit/autopilot/airspeed",                    "readonly")
dataref("fdr_ap_spd_mach","sim/cockpit/autopilot/airspeed_is_mach",            "readonly")
dataref("fdr_ap_vs",      "sim/cockpit/autopilot/vertical_velocity",           "readonly")
dataref("fdr_fd_pitch",   "sim/cockpit2/autopilot/flight_director_pitch_deg",  "readonly")
dataref("fdr_fd_roll",    "sim/cockpit2/autopilot/flight_director_roll_deg",   "readonly")
-- AP servo outputs: what the autopilot is physically commanding to control surfaces
dataref("fdr_srv_pitch",  "sim/joystick/servo_pitch_ratio",                   "readonly")
dataref("fdr_srv_roll",   "sim/joystick/servo_roll_ratio",                    "readonly")
dataref("fdr_srv_yaw",    "sim/joystick/servo_heading_ratio",                 "readonly")
-- Zibo B737-specific FMA status (safe_dataref: only registers if aircraft has these)
safe_dataref("fdr_b738_lnav",  "laminar/B738/autopilot/lnav_status",  "readonly")
safe_dataref("fdr_b738_cmda",  "laminar/B738/autopilot/cmd_a_status", "readonly")
safe_dataref("fdr_b738_cmdb",  "laminar/B738/autopilot/cmd_b_status", "readonly")
dataref("fdr_ap_mode",    "sim/cockpit/autopilot/autopilot_mode",              "readonly")
dataref("fdr_ap_state",   "sim/cockpit/autopilot/autopilot_state",             "readonly")
dataref("fdr_ap_lat_mode","sim/cockpit2/autopilot/heading_mode",               "readonly")
dataref("fdr_ap_vrt_mode","sim/cockpit2/autopilot/altitude_mode",              "readonly")
dataref("fdr_at_enabled", "sim/cockpit2/autopilot/autothrottle_enabled",       "readonly")
dataref("fdr_at_on",      "sim/cockpit2/autopilot/autothrottle_on",            "readonly")
dataref("fdr_ap_hdg",     "sim/cockpit/autopilot/heading_mag",                 "readonly")
dataref("fdr_ap_alt",     "sim/cockpit/autopilot/altitude",                    "readonly")
dataref("fdr_ap_spd",     "sim/cockpit/autopilot/airspeed",                    "readonly")
dataref("fdr_ap_spd_mach","sim/cockpit/autopilot/airspeed_is_mach",            "readonly")
dataref("fdr_ap_vs",      "sim/cockpit/autopilot/vertical_velocity",           "readonly")
dataref("fdr_fd_pitch",   "sim/cockpit2/autopilot/flight_director_pitch_deg",  "readonly")
dataref("fdr_fd_roll",    "sim/cockpit2/autopilot/flight_director_roll_deg",   "readonly")

-- ---- Radios ----
dataref("fdr_com1",      "sim/cockpit/radios/com1_freq_hz",                           "readonly")
dataref("fdr_com2",      "sim/cockpit/radios/com2_freq_hz",                           "readonly")
dataref("fdr_nav1",      "sim/cockpit/radios/nav1_freq_hz",                           "readonly")
dataref("fdr_nav2",      "sim/cockpit/radios/nav2_freq_hz",                           "readonly")
dataref("fdr_ils_loc",   "sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot",       "readonly")
dataref("fdr_ils_gs",    "sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot",       "readonly")
dataref("fdr_dme_m",     "sim/cockpit/radios/nav1_dme_dist_m",                        "readonly")
dataref("fdr_xpdr_code", "sim/cockpit/radios/transponder_code",                       "readonly")
dataref("fdr_xpdr_mode", "sim/cockpit/radios/transponder_mode",                       "readonly")
dataref("fdr_gps_brg",   "sim/cockpit2/radios/indicators/gps_bearing_deg_mag",        "readonly")
dataref("fdr_gps_dme",   "sim/cockpit2/radios/indicators/gps_dme_distance_nm",        "readonly")
dataref("fdr_baro_set",  "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot","readonly")

-- ---- EFIS / Displays ----
dataref("fdr_efis_mode",   "sim/cockpit/switches/EFIS_map_submode",       "readonly")  -- 0=app,1=vor,2=map,3=nav,4=pln
dataref("fdr_efis_range",  "sim/cockpit/switches/EFIS_map_range_selector","readonly")
dataref("fdr_efis_wx",     "sim/cockpit/switches/EFIS_shows_weather",     "readonly")
dataref("fdr_efis_tcas",   "sim/cockpit/switches/EFIS_shows_tcas",        "readonly")
dataref("fdr_efis_arpt",   "sim/cockpit/switches/EFIS_shows_airports",    "readonly")
dataref("fdr_efis_wpt",    "sim/cockpit/switches/EFIS_shows_waypoints",   "readonly")
dataref("fdr_efis_vor",    "sim/cockpit/switches/EFIS_shows_VORs",        "readonly")
dataref("fdr_efis_ndb",    "sim/cockpit/switches/EFIS_shows_NDBs",        "readonly")
dataref("fdr_hsi_arc",     "sim/cockpit2/switches/HSI_is_arc",            "readonly")

-- ---- Cockpit Switches ----
dataref("fdr_yaw_damp",   "sim/cockpit2/switches/yaw_damper_on",     "readonly")
dataref("fdr_auto_brk",   "sim/cockpit2/switches/auto_brake_level",  "readonly")  -- 0=RTO,1=off,2-5=levels
dataref("fdr_no_smoke",   "sim/cockpit2/switches/no_smoking",        "readonly")
dataref("fdr_seat_belt",  "sim/cockpit2/switches/fasten_seat_belts", "readonly")
dataref("fdr_wiper",      "sim/cockpit2/switches/wiper_speed",       "readonly")
dataref("fdr_gnd_com",    "sim/cockpit2/switches/gnd_com_power_on",  "readonly")

-- ---- Weight & CG ----
dataref("fdr_gross_kg",   "sim/flightmodel/weight/m_total",          "readonly")
dataref("fdr_payload_kg", "sim/flightmodel/weight/m_fixed",          "readonly")
dataref("fdr_cg_z_m",     "sim/flightmodel2/misc/cg_offset_z",       "readonly")
dataref("fdr_cg_pct_mac", "sim/flightmodel2/misc/cg_offset_z_mac",   "readonly")

-- ---- Environment ----
dataref("fdr_wind_spd",  "sim/weather/wind_speed_kt",                         "readonly", 0)
dataref("fdr_wind_dir",  "sim/weather/wind_direction_degt",                   "readonly", 0)
dataref("fdr_oat",       "sim/cockpit2/temperature/outside_air_temp_degc",    "readonly")
dataref("fdr_amb_temp",  "sim/weather/temperature_ambient_c",                 "readonly")
dataref("fdr_rho",       "sim/weather/rho",                                   "readonly")

-- ---- Warnings & Annunciators ----
dataref("fdr_stall",     "sim/cockpit2/annunciators/stall_warning",            "readonly")
dataref("fdr_gpws",      "sim/cockpit2/annunciators/GPWS",                     "readonly")
dataref("fdr_ws_warn",   "sim/cockpit2/annunciators/windshear_warning_systems", "readonly")  -- 0=none,1=pred_advisory,2=pred_caution,3=pred_warn_to,4=pred_warn_app,5=reactive
dataref("fdr_master_ctn","sim/cockpit2/annunciators/master_caution",            "readonly")
dataref("fdr_master_wrn","sim/cockpit2/annunciators/master_warning",            "readonly")
dataref("fdr_ap_disc",   "sim/cockpit2/annunciators/autopilot_disconnect",      "readonly")
dataref("fdr_lo_volt",   "sim/cockpit2/annunciators/low_voltage",               "readonly")
dataref("fdr_lo_fuel",   "sim/cockpit2/annunciators/fuel_quantity",             "readonly")
dataref("fdr_lo_hyd",    "sim/cockpit2/annunciators/hydraulic_pressure",        "readonly")
dataref("fdr_sbrk_ann",  "sim/cockpit2/annunciators/speedbrake",                "readonly")
dataref("fdr_ice_ann",   "sim/cockpit2/annunciators/ice",                       "readonly")

-- ---- Lights ----
dataref("fdr_lt_beacon", "sim/cockpit/electrical/beacon_lights_on",  "readonly")
dataref("fdr_lt_nav",    "sim/cockpit/electrical/nav_lights_on",     "readonly")
dataref("fdr_lt_strobe", "sim/cockpit/electrical/strobe_lights_on",  "readonly")
dataref("fdr_lt_land",   "sim/cockpit/electrical/landing_lights_on", "readonly")
dataref("fdr_lt_taxi",   "sim/cockpit/electrical/taxi_light_on",     "readonly")

-- ---- Sim State ----
dataref("fdr_sim_time",  "sim/time/total_running_time_sec", "readonly")
dataref("fdr_zulu_sec",  "sim/time/zulu_time_sec",          "readonly")
dataref("fdr_paused",    "sim/time/paused",                 "readonly")

-- ================================================================
-- FMA (Flight Mode Annunciator) label lookup tables
-- ================================================================
local LAT_MODE = {
    [0]="ROLL",      [1]="HDG SEL",   [2]="NAV",        [10]="TO/GA",
    [11]="RE-ENTRY", [12]="FREE",     [13]="GPSS",      [14]="HDG HOLD",
    [15]="TURN RATE",[16]="ROLLOUT",  [18]="TRACK"
}
local VRT_MODE = {
    [3]="PITCH",     [4]="V/S",       [5]="LVL CHG",    [6]="ALT HOLD",
    [7]="TERRAIN",   [8]="G/S",       [9]="VNAV PATH",  [10]="TO/GA",
    [11]="RE-ENTRY", [12]="FREE",     [17]="FLARE",     [19]="FPA",
    [20]="VNAV SPD"
}
local GPWS_MSG = {} -- reserved for future use if GPWS message DataRef becomes available
local WS_LEVEL = {
    [0]="NONE", [1]="PRED ADVISORY", [2]="PRED CAUTION",
    [3]="PRED WARNING T/O", [4]="PRED WARNING APPROACH", [5]="REACTIVE WARNING"
}

-- ================================================================
-- Minimal JSON encoder
-- ================================================================
local function json(v)
    local t = type(v)
    if v == nil then return "null"
    elseif t == "boolean" then return tostring(v)
    elseif t == "number" then
        if v ~= v or v == math.huge or v == -math.huge then return "null" end
        return string.format("%.7g", v)
    elseif t == "string" then
        return '"' .. v:gsub('\\','\\\\'):gsub('"','\\"'):gsub('\n','\\n'):gsub('\r','\\r'):gsub('\t','\\t') .. '"'
    elseif t == "table" then
        if #v > 0 then
            local parts = {}
            for i = 1, #v do parts[i] = json(v[i]) end
            return "[" .. table.concat(parts, ",") .. "]"
        else
            local parts = {}
            for k, val in pairs(v) do
                parts[#parts+1] = '"' .. tostring(k) .. '":' .. json(val)
            end
            return "{" .. table.concat(parts, ",") .. "}"
        end
    end
    return "null"
end

-- ================================================================
-- Snapshot builder
-- ================================================================
local function build_snapshot(now, in_critical)
    return {
        timestamp_unix  = now,
        timestamp_iso   = os.date("!%Y-%m-%dT%H:%M:%SZ", now),
        sim_time_sec    = fdr_sim_time,
        zulu_sec        = fdr_zulu_sec,
        paused          = bool(fdr_paused),
        aircraft        = AIRCRAFT_FILENAME,
        -- Adaptive sampling metadata
        phase           = in_critical and "critical" or "normal",
        sample_rate_hz  = in_critical and 4 or 0.5,

        position = {
            lat_deg         = fdr_lat,
            lon_deg         = fdr_lon,
            alt_msl_m       = fdr_alt_msl_m,
            alt_msl_ft      = mul(fdr_alt_msl_m, 3.28084),
            alt_agl_m       = fdr_alt_agl_m,
            alt_agl_ft      = mul(fdr_alt_agl_m, 3.28084),
            alt_baro_ind_ft = fdr_alt_ind,
            radio_alt_ft    = fdr_ra_ft,
            mag_variation   = fdr_mag_var,
        },

        attitude = {
            pitch_deg             = fdr_pitch,
            roll_deg              = fdr_roll,
            true_hdg_deg          = fdr_true_hdg,
            mag_hdg_deg           = fdr_mag_hdg,
            flight_path_angle_deg = fdr_vpath,
            track_true_deg        = fdr_hpath,
        },

        speed = {
            ias_kts             = fdr_ias,
            ias_ind_kts         = fdr_ias_ind,
            ias_trend_kts_per_s = fdr_ias_trend,
            tas_kts             = mul(fdr_tas_ms, 1.94384),
            gs_kts              = mul(fdr_gs_ms,  1.94384),
            vs_fpm              = fdr_vs_fpm,
            vvi_ind_fpm         = fdr_vvi_ind,
            mach                = fdr_mach,
        },

        acceleration = {
            g_normal  = fdr_g_normal,
            g_axial   = fdr_g_axial,
            g_lateral = fdr_g_lat,
        },

        aerodynamics = {
            aoa_deg        = fdr_aoa,
            sideslip_deg   = fdr_sideslip,
            slip_ratio     = fdr_slip,
            roll_rate_dps  = fdr_roll_rate,
            pitch_rate_dps = fdr_pitch_rate,
            yaw_rate_dps   = fdr_yaw_rate,
            turn_rate_dps  = fdr_turnrate,
            cl             = fdr_cl,
            cd             = fdr_cd,
            lift_n         = fdr_lift_n,
            drag_n         = fdr_drag_n,
        },

        controls = {
            yoke_pitch     = fdr_yoke_pitch,
            yoke_roll      = fdr_yoke_roll,
            -- NOTE: rudder_pedal uses yoke_heading_ratio DataRef — it IS the pedal axis
            rudder_pedal   = fdr_yoke_yaw,
            brake_left     = fdr_brake_l,
            brake_right    = fdr_brake_r,
            speedbrake_cmd = fdr_speedbrake,
            speedbrake_act = fdr_sbrkrat,
            flap_handle    = fdr_flap_handle,
            flap_actual    = fdr_flap_actual,
            slat_actual    = fdr_slat_actual,
            gear_handle    = fdr_gear_handle,
            park_brake     = fdr_park_brake,
            elev_trim      = fdr_elev_trim,
            ail_trim       = fdr_ail_trim,
            rud_trim       = fdr_rud_trim,
        },

        surfaces = {
            -- Actual surface positions (compare with pilot inputs in controls{} above)
            elevator_left_deg  = fdr_elev_l,  -- positive = TE down (nose down)
            elevator_right_deg = fdr_elev_r,
            aileron_left_deg  = fdr_lail_def,
            aileron_right_deg = fdr_rail_def,
            rudder_left_deg   = fdr_lrud_def,
            rudder_right_deg  = fdr_rrud_def,
            spoiler_left_deg  = fdr_lspl_def,
            spoiler_right_deg = fdr_rspl_def,
        },

        gear = {
            nose_deploy    = fdr_gear_d0,
            left_deploy    = fdr_gear_d1,
            right_deploy   = fdr_gear_d2,
            nose_on_ground = bool(fdr_gear_g0),
            left_on_ground = bool(fdr_gear_g1),
            rght_on_ground = bool(fdr_gear_g2),
            any_on_ground  = bool(fdr_on_ground),
            -- Nosewheel steering (critical for runway excursion analysis)
            nose_steer_cmd_deg = fdr_steer_cmd,
            nose_steer_act_deg = fdr_steer_act,
            -- Brake energy absorbed (Joules, proxy for temperature; heats up, cools in air)
            brake_energy_J     = { fdr_brk_J0, fdr_brk_J1, fdr_brk_J2 },
            brake_fade_ratio   = { fdr_brk_rat0, fdr_brk_rat1 },
            -- Tire skid: 0=full traction, 1=fully skidding (non-zero = anti-skid has activated)
            tire_skid_ratio    = { fdr_skid0, fdr_skid1, fdr_skid2 },
        },

        engines = {
            {
                throttle_ratio   = fdr_thr0,
                throttle_rev_ratio = fdr_thrr0,
                n1_pct           = fdr_n1_0,
                n2_pct           = fdr_n2_0,
                egt_c            = fdr_egt0,
                itt_c            = fdr_itt0,
                epr              = fdr_epr0,
                ff_kg_s          = fdr_ff0,
                oil_press_psi    = fdr_oilp0,
                oil_temp_c       = fdr_oilt0,
                thrust_n         = fdr_thr_n0,
                running          = bool(fdr_eon0),
                fire             = bool(fdr_efi0),
                reverser_on            = bool(fdr_rev0),
                reverser_deploy_ratio  = fdr_rev_d0,  -- 0=stowed, 1.0=fully deployed
                -- Start switches
                master_sw        = bool(fdr_emast0),
                ignition         = fdr_ign0,
                ignition_key     = fdr_ikey0,
                starter_engaged  = bool(fdr_strt0),
                fadec_on         = bool(fdr_fadec0),
                eng_fuel_pump    = fdr_epump0,
            },
            {
                throttle_ratio   = fdr_thr1,
                throttle_rev_ratio = fdr_thrr1,
                n1_pct           = fdr_n1_1,
                n2_pct           = fdr_n2_1,
                egt_c            = fdr_egt1,
                itt_c            = fdr_itt1,
                epr              = fdr_epr1,
                ff_kg_s          = fdr_ff1,
                oil_press_psi    = fdr_oilp1,
                oil_temp_c       = fdr_oilt1,
                thrust_n         = fdr_thr_n1,
                running          = bool(fdr_eon1),
                fire             = bool(fdr_efi1),
                reverser_on            = bool(fdr_rev1),
                reverser_deploy_ratio  = fdr_rev_d1,
                master_sw        = bool(fdr_emast1),
                ignition         = fdr_ign1,
                ignition_key     = fdr_ikey1,
                starter_engaged  = bool(fdr_strt1),
                fadec_on         = bool(fdr_fadec1),
                eng_fuel_pump    = fdr_epump1,
            },
        },

        eng_mode_selector = fdr_eng_mode,  -- 0=Norm, -1=Crank, 1=Ign/Start, 2=Air

        apu = {
            starter_sw  = fdr_apu_sw,     -- 0=off,1=on,2=start
            n1_pct      = fdr_apu_n1,
            egt_c       = fdr_apu_egt,
            running     = bool(fdr_apu_run),
            generator   = bool(fdr_apu_gen),
            gen_amps    = fdr_apu_amps,
            door_ratio  = fdr_apu_door,
        },

        electrical = {
            bat0_on     = bool(fdr_bat0),
            bat1_on     = bool(fdr_bat1),
            bat0_volts  = fdr_bat_v0,
            bat1_volts  = fdr_bat_v1,
            bat0_amps   = fdr_bat_a0,
            bat1_amps   = fdr_bat_a1,
            gen0_on     = bool(fdr_gen0),
            gen1_on     = bool(fdr_gen1),
            gen0_amps   = fdr_gen_a0,
            gen1_amps   = fdr_gen_a1,
            gen0_volts  = fdr_gen_v0,
            gen1_volts  = fdr_gen_v1,
            gpu_on      = bool(fdr_gpu_gen),
            gpu_volts   = fdr_gpu_v,
            gpu_avail   = bool(fdr_gpu_avail),
            avionics    = bool(fdr_avion),
            cross_tie   = bool(fdr_cross_tie),
            bus_volts   = { fdr_bus_v0, fdr_bus_v1, fdr_bus_v2, fdr_bus_v3 },
        },

        fuel = {
            total_kg       = fdr_fuel_total,
            tanks_kg       = { fdr_fuel0, fdr_fuel1, fdr_fuel2 },
            indicated_kg   = { fdr_fuelq0, fdr_fuelq1, fdr_fuelq2 },
            tank_pumps     = { bool(fdr_tank_pump0), bool(fdr_tank_pump1), bool(fdr_tank_pump2) },
            crossfeed      = fdr_xfeed,
            auto_crossfeed = fdr_xfeed_auto,
            firewall_l     = bool(fdr_fw_l),
            firewall_r     = bool(fdr_fw_r),
            selector_l     = fdr_sel_l,
            selector_r     = fdr_sel_r,
        },

        weights = {
            gross_kg   = fdr_gross_kg,
            gross_lb   = mul(fdr_gross_kg, 2.20462),
            payload_kg = fdr_payload_kg,
            zfw_kg     = type(fdr_gross_kg)=="number" and type(fdr_fuel_total)=="number"
                          and fdr_gross_kg - fdr_fuel_total or nil,
            cg_z_m     = fdr_cg_z_m,
            cg_pct_mac = fdr_cg_pct_mac,
        },

        anti_ice = {
            engine_inlet_0  = bool(fdr_ai_eng0),
            engine_inlet_1  = bool(fdr_ai_eng1),
            wing_left       = bool(fdr_ai_wing_l),
            wing_right      = bool(fdr_ai_wing_r),
            pitot_pilot     = bool(fdr_ai_pitot_p),
            pitot_copilot   = bool(fdr_ai_pitot_c),
            pitot_standby   = bool(fdr_ai_pitot_s),
            window          = bool(fdr_ai_wnd),
            aoa_probe        = bool(fdr_ai_aoa),
            static_pilot    = bool(fdr_ai_stat_p),
            static_copilot  = bool(fdr_ai_stat_c),
            tat_probe        = bool(fdr_ai_tat),
        },

        pressurization = {
            bleed_mode      = fdr_bleed_mode,  -- 0=off,1=L,2=both,3=R,4=APU,5=auto
            cabin_alt_cmd_ft = fdr_cab_alt_cmd,
            cabin_alt_ft    = fdr_cab_alt,
            cabin_vvi_fpm   = fdr_cab_vvi,
            diff_press_psi  = fdr_cab_dp,
            outflow_valve   = fdr_outflow_v,
            air_cond        = bool(fdr_air_cond),
        },

        hydraulics = {
            elec_pump1      = bool(fdr_hyd1),
            elec_pump2      = bool(fdr_hyd2),
            -- System pressures (typical: ~3000 PSI operational; units set by Plane-Maker)
            sys1_pressure   = fdr_hyd_p1,
            sys2_pressure   = fdr_hyd_p2,
            sys3_pressure   = fdr_hyd_p3,
            sys1_fluid_ratio = fdr_hyd_f1,
            sys2_fluid_ratio = fdr_hyd_f2,
            brake_acc_ratio  = fdr_hyd_brkacc,  -- brake accumulator charge 0..1
            ptu_status       = fdr_ptu,          -- 0=Off, 1=Armed, 2=Running
        },

        autopilot = {
            master_mode   = fdr_ap_mode,
            state_flags   = fdr_ap_state,
            lateral_mode  = fdr_ap_lat_mode,
            vertical_mode = fdr_ap_vrt_mode,
            -- FMA labels (human-readable, same data as lateral/vertical_mode above)
            fma_roll      = LAT_MODE[fdr_ap_lat_mode],
            fma_pitch     = VRT_MODE[fdr_ap_vrt_mode],
            at_enabled    = fdr_at_enabled,
            at_active     = bool(fdr_at_on),
            hdg_sel_deg   = fdr_ap_hdg,
            alt_sel_ft    = fdr_ap_alt,
            spd_sel       = fdr_ap_spd,
            spd_is_mach   = bool(fdr_ap_spd_mach),
            vs_sel_fpm    = fdr_ap_vs,
            fd_pitch_deg  = fdr_fd_pitch,
            fd_roll_deg   = fdr_fd_roll,
            -- AP servo outputs: what autopilot is actually commanding to control surfaces
            -- Compare with pilot inputs in controls{} to see who is "flying"
            servo_pitch   = fdr_srv_pitch,
            servo_roll    = fdr_srv_roll,
            servo_yaw     = fdr_srv_yaw,
            -- Zibo B737-specific
            b738_lnav     = fdr_b738_lnav,
            b738_cmd_a    = fdr_b738_cmda,
            b738_cmd_b    = fdr_b738_cmdb,
        },

        radios = {
            com1_hz      = fdr_com1,
            com2_hz      = fdr_com2,
            nav1_hz      = fdr_nav1,
            nav2_hz      = fdr_nav2,
            ils_loc_dots = fdr_ils_loc,
            ils_gs_dots  = fdr_ils_gs,
            dme_nm       = div(fdr_dme_m, 1852.0),
            xpdr_code    = fdr_xpdr_code,
            xpdr_mode    = fdr_xpdr_mode,
            gps_brg_deg  = fdr_gps_brg,
            gps_dist_nm  = fdr_gps_dme,
            baro_inhg    = fdr_baro_set,
        },

        efis = {
            map_mode        = fdr_efis_mode,   -- 0=app,1=vor,2=map,3=nav,4=pln
            map_range_idx   = fdr_efis_range,
            show_weather    = bool(fdr_efis_wx),
            show_tcas       = bool(fdr_efis_tcas),
            show_airports   = bool(fdr_efis_arpt),
            show_waypoints  = bool(fdr_efis_wpt),
            show_vors       = bool(fdr_efis_vor),
            show_ndbs       = bool(fdr_efis_ndb),
            hsi_arc_mode    = bool(fdr_hsi_arc),
        },

        switches = {
            yaw_damper    = bool(fdr_yaw_damp),
            auto_brake    = fdr_auto_brk,     -- 0=RTO,1=off,2-5=1-4
            no_smoking    = bool(fdr_no_smoke),
            seat_belts    = bool(fdr_seat_belt),
            wiper_speed   = fdr_wiper,
            gnd_com_power = bool(fdr_gnd_com),
        },

        environment = {
            wind_spd_ms       = fdr_wind_spd,
            wind_dir_deg      = fdr_wind_dir,
            oat_c             = fdr_oat,
            amb_temp_c        = fdr_amb_temp,
            air_density_kg_m3 = fdr_rho,
        },

        warnings = {
            stall                = bool(fdr_stall),
            gpws                 = bool(fdr_gpws),
            -- Windshear: 0=none, 1=pred_advisory, 2=pred_caution,
            --   3=pred_warning_TO, 4=pred_warning_approach, 5=reactive_warning
            windshear_level      = fdr_ws_warn,
            windshear_label      = WS_LEVEL[fdr_ws_warn],   -- human-readable label
            master_caution       = bool(fdr_master_ctn),
            master_warning       = bool(fdr_master_wrn),
            autopilot_disconnect = bool(fdr_ap_disc),
            low_voltage          = bool(fdr_lo_volt),
            low_fuel             = bool(fdr_lo_fuel),
            hydraulic_pressure   = bool(fdr_lo_hyd),
            speedbrake           = bool(fdr_sbrk_ann),
            ice                  = bool(fdr_ice_ann),
        },

        lights = {
            beacon  = bool(fdr_lt_beacon),
            nav     = bool(fdr_lt_nav),
            strobe  = bool(fdr_lt_strobe),
            landing = bool(fdr_lt_land),
            taxi    = bool(fdr_lt_taxi),
        },
    }
end

-- ================================================================
-- Main recording function — pcall-protected
-- ================================================================
function fdr_record()
    local sim_t = TOTAL_RUNNING_TIME_SEC
    -- Critical phase: low altitude (< 300 ft RA) AND moving (> 40 kts IAS)
    local in_critical = (fdr_ra_ft < 300 and fdr_ias > 40)
    local interval    = in_critical and INTERVAL_CRITICAL or INTERVAL_NORMAL
    if sim_t - last_write_time < interval then return end
    last_write_time = sim_t

    local ok, result = pcall(build_snapshot, os.time(), in_critical)
    if not ok then
        print("[FDR] snapshot error: " .. tostring(result))
        return
    end

    local f = io.open(session_file, "a")
    if f then
        f:write(json(result) .. "\n")
        f:close()
    end
end

do_every_frame("fdr_record()")

print("[FDR] started => " .. session_file)
