#include <cstdint>  // std::uint8_t

#ifndef HRSDK_HRSDK_EXPERT_H_
#define HRSDK_HRSDK_EXPERT_H_

#ifdef _WIN32
#ifdef HRSDK_API_EXPORT
#define HRSDK_API __declspec(dllexport)
#else
#define HRSDK_API __declspec(dllimport)
#endif
#else
#include <vector>
#define HRSDK_API
#define __stdcall
#define _W64
#endif

typedef int HROBOT;
#ifdef __cplusplus
extern "C" {
#endif

#define HIWIN_DBG 0
#define IS_TP03 0
enum CommandType {
	kGet = 0,
	kSet,
	kMonitorSet,
};
enum ConnectionLevels {
	kVerMismatch = -2,
	kDisconnection = -1,
	kMonitor = 0,
	kController,
	kControllerDbg,
	kControllerFile
};

enum OperationModes {
	kManual = 0,
	kAuto
};

enum LogLevels {
	kNone = 0,
	kInfo,
	kSetCommand,
	kConsole,
	kSave,
};

enum LogMessage {
	kNormal = 0,
	kWrite,
};

enum Connect {
	INVALID_CALLBACK = -2,
	CONNECT_SERVER_FAILED = -3,
	VERSION_MISMATCH = -4
};

enum SpaceOperationTypes {
	kCartesian = 0,
	kJoint,
	kTool,
	kExt
};

enum SpaceOperationDirection {
	kPositive = 1,
	kNegative = -1,
};

enum JointCoordinates {
	kJoint1 = 0,
	kJoint2,
	kJoint3,
	kJoint4,
	kJoint5,
	kJoint6
};

enum CartesianCoordinates {
	kCartesianX = 0,
	kCartesianY,
	kCartesianZ,
	kCartesianA,
	kCartesianB,
	kCartesianC
};

enum ToolCoordinates {
	kTx = 0,
	kTy,
	kTz,
	kRTx,
	kRTy,
	kRTz
};

enum RobotMotionStatus {
	kIdle = 1,
	kRunning,
	kHold,
	kDelay,
	kWait
};
enum Welder {
	Binzel_MIG_Arc_350_RS,
	Hero_TIG_RA300,
	General_MIG_Volts_WFS,
	General_MIG_Volts_Amps,
	General_TIG_Amps,
	General_TIG_Amps_WFS,
};
struct WeldSystemParameter {
	Welder welder;
	int weld_enable;
	int gas_purge_input_index;
	int gas_purge_input_type;
	double gas_purge_time;
	int torch_collision_input_index;
	int torch_collision_input_type;
	int torch_collision_input_switch_type;
	int wirestick_detection;
	double arc_start_error_time;
	double arc_loss_error_time;
	double arc_detect_time;
	short arc_retry_count;
	double arc_retry_wire_retract_time;
	int ALC_enable;
};

enum WeldMode {
	Standard,
	PulsedArc,
	JobMode,
	LowSpatterMode
};

enum WeldModeHeroTIG {
	Hero_TIG_DC,
	Hero_TIG_AC_Hard,
	Hero_TIG_AC_Standard,
	Hero_TIG_AC_Soft,
	Hero_TIG_AC_Mix,
	Hero_TIG_JobMode
};
struct WeldProcedureParameter {
	int schedule_num;
	WeldMode weld_mode;
	WeldModeHeroTIG tig_weld_mode;
	int weld_program;
	int synergy_curve;
	double runin_time;
	double runin_voltage;
	double runin_current;
	double runin_wire_speed;
	double burnback_time;
	double burnback_voltage;
	double burnback_current;
	double burnback_wire_speed;
	double crater_time;
	double crater_voltage;
	double crater_current;
	double crater_wire_speed;
	double preflow_time;
	double postflow_time;
	double tig_rise_time;
	double tig_arc_current;
	double tig_arc_time;
	double tig_forward_blowing_time;
	double tig_start_current;
	double tig_ac_dc_mix_freq;
	double tig_ac_frequence;
	double tig_negative_current_duty_cycle;
	double tig_negative_current_percentage;
	double tig_impulse_on;
	double tig_backward_blowing_time;
	double tig_final_current;
	double tig_drop_time;
	int torch_mode;
	int job_number;
};
struct WeldScheduleParameter {
	double voltage;
	double current;
	double wire_speed;
	double time;
	double tig_back_ground_main_current;
	double tig_impulse_freq;
	double tig_impulse_work_cycle;
	double tig_main_arc_current;
	double speed;
};
#define DWELL_MAX_TYPE_SIZE	2
typedef struct {
	short weaving_mode;
	short dwell_mode;
	double frequency;		// unit:Hz
	double amplitude;		// unit:mm
	double dwell_time[DWELL_MAX_TYPE_SIZE];	// unit:sec
	double elevation_angle;	// unit:deg
	double azimuth_angle;	// unit:deg
	double center_rise;		// unit:mm
	double radius;			// unit:mm
	double L_angle;			// unit:deg
	unsigned char weaving_blend;
	short  peak_digital_output_number;
	double peak_output_pulse;	// unit:sec
	double peak_output_shift;	// unit:sec
} ROBOT_WEAVING_CONDITION;
struct RefreshUIParameter {
	unsigned char  base_;
	unsigned char  counter;
	unsigned char  timer;
	unsigned char  user_alarm;
	unsigned char  pns;
	unsigned char  dio_setting;
	unsigned char  external_axis;
	unsigned char  fieldbus;
	unsigned char  moudle_io;
	unsigned char  pr;
	unsigned char  payload;
	unsigned char  ref_position;
	unsigned char  socket_info;
	unsigned char  tool;
	unsigned char  weave;
	unsigned char  dio_comment;
};

/* Connection Command */
typedef void(__stdcall *callback_function)(uint16_t, uint16_t, uint16_t *, int);
HRSDK_API HROBOT __stdcall open_connection(const char *address, int level, callback_function f);
HRSDK_API HROBOT __stdcall open_connection_no_callback(const char *address, int level);
HRSDK_API HROBOT __stdcall open_multi_connection(const char *address, int Mode, callback_function callback_func, int hrs_cnt);
HRSDK_API int __stdcall disconnect(HROBOT s);
HRSDK_API void __stdcall close_connection(HROBOT robot);
HRSDK_API int __stdcall set_connection_level(HROBOT s, int Mode);
HRSDK_API int __stdcall get_connection_level(HROBOT s);
HRSDK_API int __stdcall get_hrsdk_version(char *version);
HRSDK_API int __stdcall set_log_level(HROBOT robot_handle, LogLevels log_level);
HRSDK_API LogLevels __stdcall get_log_level(HROBOT robot_handle);

/* Register Command */
HRSDK_API int __stdcall set_timer(HROBOT s, int index, int time);
HRSDK_API int __stdcall get_timer(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_start(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_stop(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_name(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall get_timer_name(HROBOT s, int index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_timer_status(HROBOT s, int index);
HRSDK_API int __stdcall set_counter(HROBOT s, int index, int co);
HRSDK_API int __stdcall get_counter(HROBOT s, int index);
HRSDK_API int __stdcall set_pr_type(HROBOT s, int prNum, int coorType);
HRSDK_API int __stdcall get_pr_type(HROBOT s, int prNum);
HRSDK_API int __stdcall set_pr_coordinate(HROBOT s, int prNum, double *coor);
HRSDK_API int __stdcall get_pr_coordinate(HROBOT s, int pr, double *coor);
HRSDK_API int __stdcall set_pr_tool_base(HROBOT s, int prNum, int toolNum, int baseNum);
HRSDK_API int __stdcall get_pr_tool_base(HROBOT s, int pr, int *tool_base);
HRSDK_API int __stdcall set_pr(HROBOT s, int prNum, int coorType, double *coor, double *ext_pos, int tool, int base);
HRSDK_API int __stdcall get_pr(HROBOT s, int pr_num, int *coor_type, double *coor, double *ext_pos, int *tool, int *base);
HRSDK_API int __stdcall remove_pr(HROBOT s, int pr_num);
HRSDK_API int __stdcall set_pr_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall get_pr_comment(HROBOT s, int index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_real(HROBOT s, int index, double co);
HRSDK_API double __stdcall get_real(HROBOT s, int index);
HRSDK_API int __stdcall get_real_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_real_name(HROBOT s, int index, wchar_t *name);
HRSDK_API int __stdcall set_string(HROBOT s, int index, wchar_t *value);
HRSDK_API int __stdcall get_string(HROBOT s, int index, wchar_t *value, int arr_size);
HRSDK_API int __stdcall get_string_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_string_name(HROBOT s, int index, wchar_t *name);

/* System Variable Command */
HRSDK_API int __stdcall set_acc_dec_ratio(HROBOT s, int acc);
HRSDK_API int __stdcall get_acc_dec_ratio(HROBOT s);
HRSDK_API int __stdcall set_acc_time(HROBOT s, double value);
HRSDK_API double __stdcall get_acc_time(HROBOT s);
HRSDK_API int __stdcall set_ptp_speed(HROBOT s, int vel);
HRSDK_API int __stdcall get_ptp_speed(HROBOT s);
HRSDK_API int __stdcall set_lin_speed(HROBOT s, double vel);
HRSDK_API double __stdcall get_lin_speed(HROBOT s);
HRSDK_API int __stdcall set_override_ratio(HROBOT s, int vel);
HRSDK_API int __stdcall get_override_ratio(HROBOT s);
HRSDK_API int __stdcall get_robot_id(HROBOT s, char *robot_id);
HRSDK_API int __stdcall set_robot_id(HROBOT s, char *robot_id);
HRSDK_API int __stdcall set_smooth_length(HROBOT s, double r);
HRSDK_API int __stdcall get_alarm_code(HROBOT s, int &count, uint64_t *alarm_code);
HRSDK_API int __stdcall get_continuous_turn(HROBOT s, bool &enable);
HRSDK_API int __stdcall set_continuous_turn(HROBOT s, bool enable);
HRSDK_API int __stdcall set_parameter_amplification(HROBOT s, bool enable, double amplification_value);
/* Input and Output Command */
HRSDK_API int __stdcall get_digital_input(HROBOT s, int index);
HRSDK_API int __stdcall get_digital_output(HROBOT s, int index);
HRSDK_API int __stdcall set_digital_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_robot_input(HROBOT s, int index);
HRSDK_API int __stdcall get_robot_output(HROBOT s, int index);
HRSDK_API int __stdcall set_robot_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_valve_output(HROBOT s, int index);
HRSDK_API int __stdcall set_valve_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_function_input(HROBOT s, int index);
HRSDK_API int __stdcall get_function_output(HROBOT s, int index);
HRSDK_API int __stdcall get_function_input_sim(HROBOT s, int index, bool &value);
HRSDK_API int __stdcall get_function_output_sim(HROBOT s, int index, bool &value);
HRSDK_API int __stdcall set_function_input_sim(HROBOT s, int index, bool value);
HRSDK_API int __stdcall set_function_output_sim(HROBOT s, int index, bool value);
HRSDK_API int __stdcall set_function_input_value(HROBOT s, int index, bool value);
HRSDK_API int __stdcall set_function_output_value(HROBOT s, int index, bool value);
HRSDK_API int __stdcall get_system_input(HROBOT robot, int index, bool &value, char *comment);
HRSDK_API int __stdcall get_system_output(HROBOT robot, int index, bool &value, char *comment);

/* Coordinate System Command */
HRSDK_API int __stdcall set_base_number(HROBOT s, int state);
HRSDK_API int __stdcall get_base_number(HROBOT s);
HRSDK_API int __stdcall define_base(HROBOT s, int baseNum, double *coor);
HRSDK_API int __stdcall get_base_data(HROBOT s, int num, double *coor);
HRSDK_API int __stdcall get_moving_base_data(HROBOT s, double *coor);
HRSDK_API int __stdcall set_base_name(HROBOT s, int index, wchar_t *name);
HRSDK_API int __stdcall get_base_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_tool_number(HROBOT s, int num);
HRSDK_API int __stdcall get_tool_number(HROBOT s);
HRSDK_API int __stdcall define_tool(HROBOT s, int toolNum, double *coor);
HRSDK_API int __stdcall get_tool_data(HROBOT s, int num, double *coor);
HRSDK_API int __stdcall set_tool_name(HROBOT s, int index, wchar_t *name);
HRSDK_API int __stdcall get_tool_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall clear_base_data(HROBOT s, int index);
HRSDK_API int __stdcall clear_tool_data(HROBOT s, int index);

/* Task Command */
HRSDK_API int __stdcall set_rsr(HROBOT s, char *filename, int index);
HRSDK_API int __stdcall get_rsr_prog_name(HROBOT s, int rsr_index, char *file_name);
HRSDK_API int __stdcall remove_rsr(HROBOT s, int index);
HRSDK_API int __stdcall ext_task_start(HROBOT s, int mode, int select);
HRSDK_API int __stdcall task_start(HROBOT s, char *task_name);
HRSDK_API int __stdcall task_hold(HROBOT s);
HRSDK_API int __stdcall task_continue(HROBOT s);
HRSDK_API int __stdcall task_abort(HROBOT s);
HRSDK_API int __stdcall task_abort_wait_finish(HROBOT s);
HRSDK_API int __stdcall get_execute_file_name(HROBOT s, char *filename);
HRSDK_API int __stdcall task_skip_line(HROBOT s, char *task_name);
HRSDK_API int __stdcall set_skip_line(HROBOT s, int line, char *filename);

/* File Command */
HRSDK_API int __stdcall send_file(HROBOT sock, char *fromFilePath, char *toFilePath);
HRSDK_API int __stdcall download_file(HROBOT s, char *fromFilePath, char *toFilePath);

/* Controller Setting Command */
HRSDK_API int __stdcall get_hrss_mode(HROBOT s);
HRSDK_API int __stdcall get_robot_info(HROBOT s, int page_sel, int tool_num, int base_num, char *info, bool is_ready);
HRSDK_API int __stdcall set_motor_state(HROBOT s, int state);
HRSDK_API int __stdcall get_motor_state(HROBOT s);
HRSDK_API int __stdcall set_operation_mode(HROBOT s, int mode);
HRSDK_API int __stdcall get_operation_mode(HROBOT s);
HRSDK_API int __stdcall clear_alarm(HROBOT s);
HRSDK_API int __stdcall update_hrss(HROBOT s, char *path);
HRSDK_API int __stdcall set_speed_limit(HROBOT s, bool mode);
HRSDK_API int __stdcall set_language(HROBOT s, int language_number);

/* Cartesian Robot Command*/
HRSDK_API int __stdcall set_cr4_cartesian_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall get_cr4_cartesian_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall set_cr4_motor_parameter(HROBOT s, int axisnum, int32_t pos_to_encoder_dir, int32_t ppr, int32_t rpm, double pitch, double gear_ratio, double high_limit, double low_limit, double max_rpm);
HRSDK_API int __stdcall get_cr4_motor_parameter(HROBOT s, int32_t *pos_to_encoder_dir, int32_t *ppr, int32_t *rpm, double *pitch, double *gear_ratio, double *high_limit, double *low_limit, double *max_rpm);

/* Jog */
HRSDK_API int __stdcall jog(HROBOT robot, int space_type, int index, int dir);
HRSDK_API int __stdcall jog_stop(HROBOT s);

/* Motion Command */
HRSDK_API int __stdcall ptp_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_rel_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_rel_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_pr(HROBOT s, int mode, int p);
HRSDK_API int __stdcall ptp_axis_velocity(HROBOT s, double acc_time, double ratio, int mode, double *p);
HRSDK_API int __stdcall ptp_point(HROBOT s, int mode, double *axis, double *pos, double *ext);
HRSDK_API int __stdcall lin_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_rel_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_rel_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_pr(HROBOT s, int mode, double smooth_value, int p);
HRSDK_API int __stdcall lin_pos_velocity(HROBOT s, double acc_time, double speed, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall circ_pos(HROBOT s, int mode, double *p_aux, double *p_end);
HRSDK_API int __stdcall circ_axis(HROBOT s, int mode, double *p_aux, double *p_end);
HRSDK_API int __stdcall circ_pr(HROBOT s, int mode, int p1, int p2);
HRSDK_API int __stdcall motion_hold(HROBOT s);
HRSDK_API int __stdcall motion_continue(HROBOT s);
HRSDK_API int __stdcall motion_abort(HROBOT s);
HRSDK_API int __stdcall motion_delay(HROBOT s, int delay);
HRSDK_API int __stdcall set_command_id(HROBOT s, int id);
HRSDK_API int __stdcall get_command_id(HROBOT s);
HRSDK_API int __stdcall get_command_count(HROBOT s);
HRSDK_API int __stdcall get_motion_state(HROBOT s);
HRSDK_API int __stdcall remove_command(HROBOT s, int num);
HRSDK_API int __stdcall remove_command_tail(HROBOT s, int num);
HRSDK_API int __stdcall motion_reachable(HROBOT s, double *dest_coord, bool &is_reach);
HRSDK_API int __stdcall motion_check_lin(HROBOT s, double *coord1, double *coord2, bool &is_reach);

/* Manipulator Information Command */
HRSDK_API int __stdcall get_encoder_count(HROBOT s, int32_t *EncCount);
HRSDK_API int __stdcall get_current_joint(HROBOT s, double *joint);
HRSDK_API int __stdcall get_current_position(HROBOT s, double *cart);
HRSDK_API int __stdcall get_position_b0t0(HROBOT s, double *pos);
HRSDK_API int __stdcall get_position_info(HROBOT s, double *cart, double *joints, int *encoder, double *ext_pos, int *ext_encoder);
HRSDK_API int __stdcall get_current_rpm(HROBOT s, double *rpm);
HRSDK_API int __stdcall get_current_ext_rpm(HROBOT s, double *rpm);
HRSDK_API int __stdcall get_current_tcp_speed(HROBOT s, double &speed);
HRSDK_API int __stdcall get_device_born_date(HROBOT s, int *YMD);
HRSDK_API int __stdcall get_operation_time(HROBOT s, int *YMDHm);
HRSDK_API int __stdcall get_mileage(HROBOT s, double *mil);
HRSDK_API int __stdcall get_total_mileage(HROBOT s, double *tomil);
HRSDK_API int __stdcall get_utilization(HROBOT s, int *utl);
HRSDK_API int __stdcall get_utilization_ratio(HROBOT s, double &ratio);
HRSDK_API int __stdcall get_motor_torque(HROBOT s, double *cur);
HRSDK_API int __stdcall get_robot_type(HROBOT s, char *robType);
HRSDK_API int __stdcall get_hrss_version(HROBOT s, char *ver);
HRSDK_API int __stdcall get_position_to_joint(HROBOT s, double *nowAxis, double *destPos, double *destAxis, double *extAxisPos, int tool_num, int base_num);

/* Expert */
/* Connection Command */
HRSDK_API HROBOT __stdcall ConnectTcpServer(const char *address, int port);
HRSDK_API HROBOT __stdcall ConnectEx(const char *address, int level, callback_function f);

/* Register Command */

/* System Variable Command */
HRSDK_API int __stdcall get_error_msg(HROBOT s, char *msg);

/* Input and Output Command */
HRSDK_API int __stdcall GetRIOInputValue(HROBOT s);
HRSDK_API int __stdcall GetRIOInputValueEx(HROBOT s, std::uint8_t group);
HRSDK_API int __stdcall SetRIOOutputValueByNumber(HROBOT s, int on_off, int index);
HRSDK_API int __stdcall GetRIOOutputValue(HROBOT s);
HRSDK_API int __stdcall GetRIOOutputValueEx(HROBOT s, std::uint8_t group);
HRSDK_API int __stdcall SetRIOVOutputValueByNumber(HROBOT s, int on_off, int VOx);
HRSDK_API int __stdcall GetRIOVOutputValue(HROBOT s);
HRSDK_API int __stdcall GetRIOVOutputValueEx(HROBOT s, std::uint8_t group);
HRSDK_API int __stdcall SyncOutput(HROBOT s, int O_type, int O_id, int on_off, int synMode, int delay, int distance);
HRSDK_API int __stdcall set_RI_simulation_Enable(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_RI_simulation(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_RI_simulation_Enable(HROBOT s, int index);
HRSDK_API int __stdcall set_robot_input_comment(HROBOT s, int ri_index, wchar_t *comment);
HRSDK_API int __stdcall get_robot_input_comment(HROBOT s, int ri_index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_robot_output_comment(HROBOT s, int ri_index, wchar_t *comment);
HRSDK_API int __stdcall get_robot_output_comment(HROBOT s, int ri_index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_DI_simulation_Enable(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_DI_simulation(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_DI_simulation_Enable(HROBOT s, int index);
HRSDK_API int __stdcall set_digital_input_comment(HROBOT s, int di_index, wchar_t *comment);
HRSDK_API int __stdcall get_digital_input_comment(HROBOT s, int di_index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_digital_output_comment(HROBOT s, int do_index, wchar_t *comment);
HRSDK_API int __stdcall get_digital_output_comment(HROBOT s, int do_index, wchar_t *comment, int arr_size);

/* Coordinate System Command */
HRSDK_API int __stdcall tool_calibration(HROBOT s, int calibrate_type, double *p0coor, double *p1coor, double *p2coor, double *p3coor, double *result_coor);
HRSDK_API int __stdcall base_calibration(HROBOT s, int calibrate_type, double *p0coor, double *p1coor, double *p2coor, double *result_coor);
HRSDK_API int __stdcall set_home_point(HROBOT s, double *joint);
HRSDK_API int __stdcall set_ext_home_point(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall get_home_point(HROBOT s, double *joint);
HRSDK_API int __stdcall get_ext_home_point(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall get_previous_pos(HROBOT s, double *joint);
HRSDK_API int __stdcall get_previous_extpos(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall confirm_home_point(HROBOT s);

/* Task Command */
HRSDK_API int __stdcall get_prog_number(HROBOT s);
HRSDK_API int __stdcall get_prog_name(HROBOT s, int file_index, char *file_name);
HRSDK_API int __stdcall get_syntax_error_line(HROBOT s, int *line);

/* File Command */
HRSDK_API int __stdcall delete_file(HROBOT s, char *FilePath);
HRSDK_API int __stdcall delete_folder(HROBOT s, char *FilePath);
HRSDK_API int __stdcall new_folder(HROBOT s, char *FilePath);
HRSDK_API int __stdcall file_rename(HROBOT s, char *oldFilePath, char *newFilePath);
HRSDK_API int __stdcall file_drag(HROBOT s, char *fromFilePath, char *toFilePath);

#if HCROS
HRSDK_API int __stdcall save_database(HROBOT s);
#else
HRSDK_API int __stdcall save_database(HROBOT s, char *file_path);
#endif

HRSDK_API int __stdcall load_database(HROBOT s, char *file_path);
HRSDK_API int __stdcall file_copy(HROBOT s, char *oldFilePath, char *newFilePath, int opt);  // opt:0-folder.1-file

/* Controller Setting Command */
HRSDK_API int __stdcall set_server_ip(HROBOT s, int *ip);
HRSDK_API int __stdcall mastering(HROBOT s, char *joint, char *type);
HRSDK_API int __stdcall calibration(HROBOT s, char *joint, char *type);
HRSDK_API int __stdcall delete_backup_file(HROBOT s);
HRSDK_API int __stdcall network_connect(HROBOT s);
HRSDK_API int __stdcall network_disconnect(HROBOT s);
HRSDK_API int __stdcall network_send_msg(HROBOT s, char *msg);
HRSDK_API int __stdcall network_recieve_msg(HROBOT s, char *msg);
HRSDK_API int __stdcall get_network_config(HROBOT s, int &connect_type, char *ip_addr, int &port, int &bracket_type, int &separator_type, bool &is_format);
HRSDK_API int __stdcall get_network_config_mask(HROBOT s, int &connect_type, char *ip_addr, char *subnet_mask, int &port, int &bracket_type, int &separator_type, bool &is_format);
HRSDK_API int __stdcall set_network_config(HROBOT s, int connect_type, char *ip_addr, int port, int bracket_type, int separator_type, bool is_format);
HRSDK_API int __stdcall set_network_config_mask(HROBOT s, int connect_type, char *ip_addr, char *subnet_mask, int port, int bracket_type, int separator_type, bool is_format);
HRSDK_API int __stdcall network_change_ip(HROBOT s, int lan_index, int ip_type, char *ip_addr);
HRSDK_API int __stdcall network_change_ip_mask(HROBOT s, int lan_index, int ip_type, char *ip_addr, char *subnet_mask);
HRSDK_API int __stdcall network_get_state(HROBOT s);
HRSDK_API int __stdcall network_get_state(HROBOT s);
HRSDK_API int __stdcall get_lan_ip(HROBOT s, int &lan_count, char *ip_addr);
HRSDK_API int __stdcall get_lan_ip_mask(HROBOT s, int &lan_count, char *ip_addr, char *subnet_mask);

HRSDK_API int __stdcall set_cr4_return_zero_parameter(HROBOT s, int axisnum, int dir, int speed_1, int speed_2, int speed_3);
HRSDK_API int __stdcall get_cr4_return_zero_parameter(HROBOT s, int axisnum, int &dir, int &speed_1, int &speed_2, int &speed_3);
HRSDK_API int __stdcall set_cr4_return_zero_sequence(HROBOT s, int sequence);
HRSDK_API int __stdcall get_cr4_return_zero_sequence(HROBOT s, int &sequence);
HRSDK_API int __stdcall export_cr4_parameter(HROBOT s, char *file_name);
HRSDK_API int __stdcall import_cr4_parameter(HROBOT s, char *file_name);
HRSDK_API int __stdcall export_cr4_system_parameter(HROBOT s);
HRSDK_API int __stdcall import_cr4_system_parameter(HROBOT s);

HRSDK_API int __stdcall get_rs232_config(HROBOT s, int &baud_rate, int &data_bits, int &parity, int &stop_bit, int &bracket_type, int &separator_type, int &is_format);
HRSDK_API int __stdcall set_rs232_config(HROBOT s, int baud_rate, int data_bits, int parity, int stop_bit, int bracket_type, int separator_type, int is_format);
HRSDK_API int __stdcall rs232_connect(HROBOT s);
HRSDK_API int __stdcall rs232_disconnect(HROBOT s);
HRSDK_API int __stdcall rs232_get_state(HROBOT s);
HRSDK_API int __stdcall rs232_send_msg(HROBOT s, char *msg);
HRSDK_API int __stdcall clear_abs_error(HROBOT s);
HRSDK_API int __stdcall software_emergency_stop(HROBOT s);
HRSDK_API int __stdcall switch_btn_stop(HROBOT s);
HRSDK_API int __stdcall set_controller_time(HROBOT s, int year, int month, int day, int hour, int minute, int second);
HRSDK_API int __stdcall get_controller_time(HROBOT s, int &year, int &month, int &day, int &hour, int &minute, int &second);

/* Jog */
HRSDK_API int __stdcall teach_point(HROBOT s, int p);
HRSDK_API int __stdcall get_teach_point_coordinate(HROBOT s, int p, int coorType, double *coor);
HRSDK_API int __stdcall get_teach_point_tool_base(HROBOT s, int p, int *Toolbase);
HRSDK_API int __stdcall jog_home(HROBOT s);
HRSDK_API int __stdcall jog_tool_motion(HROBOT s, double *tool_offset, double *tool_offset_target);
HRSDK_API int __stdcall jog_rotation(HROBOT s, int axis, double offset);

/* Motion Command */
HRSDK_API int __stdcall lin_rel_tool(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall spline(HROBOT s, int mode, double smooth_value);
HRSDK_API int __stdcall set_spl(HROBOT s, int coor_type, double *p);
HRSDK_API int __stdcall clear_spl(HROBOT s);

/* Manipulator Information Command */
HRSDK_API int __stdcall get_robot_type_v2(HROBOT s, const char *&robType);
HRSDK_API int __stdcall get_hrss_version_v2(HROBOT s, const char *&version);
HRSDK_API int __stdcall download_log_file(HROBOT s, char *fromFilePath, char *toFilePath);
HRSDK_API int __stdcall download_history_zip(HROBOT s, char *toFilePath);
HRSDK_API int __stdcall get_battery_status(HROBOT s);
HRSDK_API int __stdcall download_caterpillar(HROBOT s, wchar_t *toFilePath);
HRSDK_API int __stdcall download_utilization_file(HROBOT s, wchar_t *toFilePath);
HRSDK_API int __stdcall compress_history_zip(HROBOT s);
HRSDK_API int __stdcall download_history_zip_no_compression(HROBOT s, wchar_t *toFilePath);
HRSDK_API int __stdcall refresh_ui(HROBOT s, RefreshUIParameter refresh_ui);
HRSDK_API int __stdcall write_operationhistory(HROBOT s, char *log_message);
/* Development */
HRSDK_API void __stdcall HRSDKDebug(HROBOT s, int op);
HRSDK_API int __stdcall show_debug_console();
HRSDK_API void __stdcall WriteLog(char *chr, int command = -99);
HRSDK_API int __stdcall get_version(char *hrs_ver_title, char *cater_ver_title, char *version);

/* unutilized */
HRSDK_API int __stdcall set_variable(HROBOT s, char *variable_name, double variable);
HRSDK_API int __stdcall get_variable(HROBOT s, char *variable_name, double *variable);
HRSDK_API int __stdcall set_pass_keypro(HROBOT s, bool value);
HRSDK_API int __stdcall get_current_pos(HROBOT s, int coorType, double *coor);
HRSDK_API int __stdcall get_driver_error_code(HROBOT s, int driverNum);
HRSDK_API int __stdcall get_motion_error_code(HROBOT s);

HRSDK_API int __stdcall set_network_show_msg(HROBOT s, int enable);
HRSDK_API int __stdcall get_network_show_msg(HROBOT s, int &flag);
HRSDK_API int __stdcall get_rs232_show_msg(HROBOT s, int &flag);
HRSDK_API int __stdcall set_rs232_show_msg(HROBOT s, int enable);
/*---fieldbus---*/
HRSDK_API int __stdcall get_fieldbus_input(HROBOT s, int *input, int index);
HRSDK_API int __stdcall get_fieldbus_input_sim(HROBOT s, int *input, int index);
HRSDK_API int __stdcall get_fieldbus_output(HROBOT s, int *input, int index);
HRSDK_API int __stdcall get_fieldbus_input_comment(HROBOT s, int index, wchar_t *comment, int arr_size, int &next);
HRSDK_API int __stdcall get_fieldbus_output_comment(HROBOT s, int index, wchar_t *comment, int arr_size, int &next);
HRSDK_API int __stdcall set_fieldbus_input_sim(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_fieldbus_input(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_fieldbus_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_fieldbus_input_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall set_fieldbus_output_comment(HROBOT s, int index, wchar_t *comment);
/*---fieldbus_rs---*/
HRSDK_API int __stdcall get_fieldbus_rs_srr(HROBOT s, int index);
HRSDK_API int __stdcall get_fieldbus_rs_srw(HROBOT s, int index);
HRSDK_API int __stdcall get_fieldbus_rs_comment(HROBOT s, int index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_fieldbus_rs_parameter(HROBOT s, int index, char *parameter);
HRSDK_API int __stdcall set_fieldbus_rs_srw(HROBOT s, int index, int value);
HRSDK_API int __stdcall set_fieldbus_rs_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall set_fieldbus_rs_parameter(HROBOT s, int index, char *parameter);
HRSDK_API int __stdcall clean_fieldbus_rs_parameter(HROBOT s, int index);
/*---cclink---*/
HRSDK_API int __stdcall get_fieldbus_connection(HROBOT s, int *is_slave_connect, int *slave_connection_type, int *slave_flag);
HRSDK_API int __stdcall get_cclink_config(HROBOT s, int slave_num, int &station_num, int &rate, int &occupancy);
HRSDK_API int __stdcall set_cclink_config(HROBOT s, int slave_num, int station_num, int rate, int occupancy);
HRSDK_API int __stdcall cclink_connection(HROBOT s, int slave_num, bool connection);
/*---profinet---*/
HRSDK_API int __stdcall get_profinet_config(HROBOT s, int slave_num, char *station_name, char *ip, int &input, int &output);
HRSDK_API int __stdcall set_profinet_config(HROBOT s, int slave_num, char *station_name, char *ip, int input, int output);
HRSDK_API int __stdcall profinet_connection(HROBOT s, int slave_num, bool connection);
/*---modbus---*/
HRSDK_API int __stdcall set_modbus_server_config(HROBOT s, char *local_ip1, char *local_ip2, int local_port);
HRSDK_API int __stdcall get_modbus_server_config(HROBOT s, char *local_ip1, char *local_ip2, int &local_port);
HRSDK_API int __stdcall get_modbus_server_config_mask(HROBOT s, char *local_ip1, char *local_ip2, char *local_mask1, char *local_mask2, int &local_port);
HRSDK_API int __stdcall set_modbus_client_config(HROBOT s, int slave_num, int server_num, char *remote_ip, int remote_port, int input_begin, int input_size, int output_begin, int output_size, int register_begin, int register_size, int in_reg_begin, int in_reg_size);
HRSDK_API int __stdcall get_modbus_client_config(HROBOT s, int slave_num, int &server_num, char *remote_ip, int &remote_port, int &input_begin, int &input_size, int &output_begin, int &output_size, int &register_begin, int &register_size, int &in_reg_begin, int &in_reg_size);
HRSDK_API int __stdcall modbus_server_connection(HROBOT s, int slave_num, bool connection);
HRSDK_API int __stdcall modbus_client_connection(HROBOT s, int slave_num, bool connection);
/*---modbus monitor---*/
HRSDK_API int __stdcall set_modbus_monitor_show_msg(HROBOT s, bool show);
HRSDK_API int __stdcall get_modbus_monitor_show_msg(HROBOT s, bool &show);
HRSDK_API int __stdcall modbus_monitor_send_msg(HROBOT s, char *show);
HRSDK_API int __stdcall set_modbus_monitor_config(HROBOT s, int channel, int server_num);
HRSDK_API int __stdcall get_modbus_monitor_config(HROBOT s, int &channel, int &server_num);
HRSDK_API int __stdcall modbus_monitor_export(HROBOT s, wchar_t *filePath);

/*---Counter-4/29--*/
HRSDK_API int __stdcall get_counter_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_counter_name(HROBOT s, int index, wchar_t *name);
/*---Track-5/2--*/
HRSDK_API int __stdcall get_track_cnv_setting_config(HROBOT s, int index, int &status, int &direction, int &trigger, int &times, int &batch, int &source);
HRSDK_API int __stdcall set_track_cnv_setting_config(HROBOT s, int index, bool is_status, int status, int direction, int trigger, int times, int batch, int source);
HRSDK_API int __stdcall save_track_cnv_setting_config(HROBOT s);
HRSDK_API int __stdcall get_track_motion_setting_config(HROBOT s, int index, int &dalay, int &acc, int &min_latch_count, int &compare_num, char *compare_dist);
HRSDK_API int __stdcall set_track_motion_setting_config(HROBOT s, int index, int dalay, int acc, int min_latch_count, int compare_num, char *compare_dist);
HRSDK_API int __stdcall save_track_motion_setting_config(HROBOT s);
HRSDK_API int __stdcall get_track_ack_setting_config(HROBOT s, int &vision, char *protocol);
HRSDK_API int __stdcall set_track_ack_setting_config(HROBOT s, int vision, char *protocol);
HRSDK_API int __stdcall save_track_ack_setting_config(HROBOT s);
HRSDK_API int __stdcall get_track_dio_setting_config(HROBOT s, int index, int &delay, int &type, int &detect_time, int &keep_time, int &strategy);
HRSDK_API int __stdcall set_track_dio_setting_config(HROBOT s, int index, bool is_status, int delay, int type, int detect_time, int keep_time, int strategy);
HRSDK_API int __stdcall save_track_dio_setting_config(HROBOT s);
HRSDK_API int __stdcall get_track_encoder(HROBOT s, uint32_t *encoder);
/*---Track_Vision-5/3---*/
HRSDK_API int __stdcall get_track_vision_setting_config(HROBOT s, int index, double &x_off, double &y_off, int &count, int &ip_chan, char *ip, int &port, int &ip_count, char *local_ip, char *encoder);
HRSDK_API int __stdcall set_track_vision_setting_config(HROBOT s, int index, double x_off, double y_off, int count, int ip_chan, char *ip, int port, char *local_ip);
HRSDK_API int __stdcall save_track_vision_setting_config(HROBOT s, int index);
HRSDK_API int __stdcall reset_track_vision_setting_config(HROBOT s, int index);
/*---Track_Object-5/6---*/
HRSDK_API int __stdcall get_track_object_config(HROBOT s, double *obj, int *tool, int *base);
HRSDK_API int __stdcall set_track_object_config(HROBOT s, int row, int column);
HRSDK_API int __stdcall save_track_object_config(HROBOT s);
HRSDK_API int __stdcall cancel_track_object_config(HROBOT s);
/*---Track_Sensor-5/6---*/
HRSDK_API int __stdcall get_track_sensor_config(HROBOT s, int *sensor);
HRSDK_API int __stdcall set_track_sensor_config(HROBOT s, int row, int column, int value);
HRSDK_API int __stdcall save_track_sensor_config(HROBOT s, int &used_num);
HRSDK_API int __stdcall cancel_track_sensor_config(HROBOT s);
/*---Track_Calibration-5/7---*/
HRSDK_API int __stdcall get_track_calibration_config(HROBOT s, int index, int &type, double *data, uint32_t *pulse);
HRSDK_API int __stdcall set_track_calibration_config(HROBOT s, int index, int mode);
HRSDK_API int __stdcall set_track_calibration_config2(HROBOT s, int index, double *pos, uint32_t *pulse);
HRSDK_API int __stdcall set_track_calibration_tab(HROBOT s, int index);
HRSDK_API int __stdcall save_track_calibration_config(HROBOT s);
HRSDK_API int __stdcall reset_track_calibration_config(HROBOT s);
/*---Track_Monitor-5/7---*/
HRSDK_API int __stdcall get_track_monitor_config(HROBOT s, int index, double &speed, int *data, uint32_t &latch, char *encoder, uint32_t *unfinish_queue, uint32_t *finish_queue);
HRSDK_API int __stdcall clear_track_monitor_config(HROBOT s, int index);
/*---Track_Start---*/
HRSDK_API int __stdcall virtual_conveyor_start(HROBOT s, int index);
HRSDK_API int __stdcall virtual_conveyor_hold(HROBOT s, int index);
HRSDK_API int __stdcall virtual_conveyor_stop(HROBOT s, int index);
HRSDK_API int __stdcall set_virtual_conveyor_speed(HROBOT s, int index, int speed);
HRSDK_API int __stdcall set_create_virtual_object_by_time(HROBOT s, int index, int millisecond);
HRSDK_API int __stdcall virtual_object_trigger_start(HROBOT s, int index);
HRSDK_API int __stdcall virtual_object_trigger_stop(HROBOT s, int index);
HRSDK_API int __stdcall trigger_obejct(HROBOT s, int index);

/*---GetExtAngle&Mode*/
HRSDK_API int __stdcall get_current_ext_pos(HROBOT s, double *pos);
HRSDK_API int __stdcall get_current_ext_mode(HROBOT s, char *mode);

/**/
HRSDK_API int __stdcall jog_rotation_increment(HROBOT s, int mode, double degree);

/**/
HRSDK_API int __stdcall ptp_multi_pos(HROBOT s, int mode, double *p, int count);

/**/
HRSDK_API int __stdcall get_program_line(HROBOT s, int &line);

/*DIO Setting*/
HRSDK_API int __stdcall get_digital_setting(HROBOT s, int *index, char *text);
HRSDK_API int __stdcall set_digital_setting(HROBOT s, int *index, char *text);

/*PNS Setting*/
HRSDK_API int __stdcall set_pns(HROBOT s, int index, char *file_path);
HRSDK_API int __stdcall get_pns(HROBOT s, int *index);
HRSDK_API int __stdcall save_pns_mode(HROBOT s, int index);
HRSDK_API int __stdcall save_pns_strobe(HROBOT s, int index);
HRSDK_API int __stdcall save_pns(HROBOT s, int type, int index_from, int index_to, int &index);
HRSDK_API int __stdcall get_pns_data(HROBOT s, int index, char *file_name);
HRSDK_API int __stdcall delete_pns_data(HROBOT s, int index);
HRSDK_API int __stdcall get_fio_mode(HROBOT s, int &mode);

/*Enter Password*/
HRSDK_API int __stdcall enter_password(HROBOT s, char *password);

/*Shutdown*/
HRSDK_API int __stdcall hrss_shutdown(HROBOT s);

/*Ecat IO*/
HRSDK_API int __stdcall get_ecat_size(HROBOT s, int &io_num, int *input_size, int *output_size);

/*Module I/O 10/16*/
HRSDK_API int __stdcall get_module_input_config(HROBOT s, int index, bool &sim, bool &value, int &type, int &start, int &end, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_module_output_config(HROBOT s, int index, bool &value, int &type, int &start, int &end, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_module_input_simulation(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_input_type(HROBOT s, int index, int type);
HRSDK_API int __stdcall set_module_input_value(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_input_start(HROBOT s, int index, int start_number);
HRSDK_API int __stdcall set_module_input_end(HROBOT s, int index, int end_number);
HRSDK_API int __stdcall set_module_input_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall set_module_output_type(HROBOT s, int index, int type);
HRSDK_API int __stdcall set_module_output_value(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_output_start(HROBOT s, int index, int start_number);
HRSDK_API int __stdcall set_module_output_end(HROBOT s, int index, int end_number);
HRSDK_API int __stdcall set_module_output_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall save_module_io_setting(HROBOT s);
HRSDK_API int __stdcall set_MI_config(HROBOT s, int index, int type, int start, int end, int &used_idx);
HRSDK_API int __stdcall set_MO_config(HROBOT s, int index, int type, int start, int end, int &used_idx);

/* Soft Limit */
HRSDK_API int __stdcall enable_joint_soft_limit(HROBOT s, bool enable);
HRSDK_API int __stdcall enable_cart_soft_limit(HROBOT s, bool enable);
HRSDK_API int __stdcall set_joint_soft_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall set_cart_soft_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall get_joint_soft_limit_config(HROBOT s, bool &enable, double *low_limit, double *high_limit);
HRSDK_API int __stdcall get_cart_soft_limit_config(HROBOT s, bool &enable, double *low_limit, double *high_limit);

/*Single Step*/
HRSDK_API int __stdcall get_single_step(HROBOT s, bool &value);
HRSDK_API int __stdcall set_single_step(HROBOT s, bool value);
HRSDK_API int __stdcall single_step(HROBOT s, bool value);

/*Payload*/
HRSDK_API int __stdcall get_payload_config(HROBOT s, int index, double *value, char *comment);
HRSDK_API int __stdcall set_payload_config(HROBOT s, int index, double *value, char *comment);
HRSDK_API int __stdcall get_payload_active(HROBOT s, int &index);
HRSDK_API int __stdcall set_payload_active(HROBOT s, int index);

/*User Alarm Setting*/
HRSDK_API int __stdcall get_user_alarm_setting_message(HROBOT s, int num, char *message);
HRSDK_API int __stdcall set_user_alarm_setting_message(HROBOT s, int num, char *message);

/*Modbus RTU 3/25*/
#if HCROS
HRSDK_API int __stdcall get_modbus_server_RTU_config(HROBOT s, int *config);
HRSDK_API int __stdcall get_modbus_client_RTU_config(HROBOT s,  int *config);
HRSDK_API int __stdcall set_modbus_server_RTU_config(HROBOT s, int *config);
HRSDK_API int __stdcall set_modbus_client_RTU_config(HROBOT s, int *config);
#else
HRSDK_API int __stdcall get_modbus_server_RTU_config(HROBOT s, int channel_num, int *config);
HRSDK_API int __stdcall get_modbus_client_RTU_config(HROBOT s, int channel_num, int *config);
HRSDK_API int __stdcall set_modbus_server_RTU_config(HROBOT s, int channel_num, int *config);
HRSDK_API int __stdcall set_modbus_client_RTU_config(HROBOT s, int channel_num, int *config);
#endif
HRSDK_API int __stdcall modbus_server_RTU_connection(HROBOT s, int channel_num, bool connection);
HRSDK_API int __stdcall modbus_client_RTU_connection(HROBOT s, int channel_num, bool connection);

/*Home Setting 4/22*/
HRSDK_API int __stdcall get_home_setting_enable(HROBOT s, bool &enable);
HRSDK_API int __stdcall set_home_setting_enable(HROBOT s, bool enable);
HRSDK_API int __stdcall get_home_confirm_enable(HROBOT s, bool &enable);

/*Electric Gripper*/
HRSDK_API int __stdcall eg_open_connection(HROBOT s, int type);
HRSDK_API int __stdcall eg_close_connection(HROBOT s);
HRSDK_API int __stdcall eg_reset(HROBOT s);
HRSDK_API int __stdcall eg_install_driver(HROBOT s, char *file_path);
HRSDK_API int __stdcall eg_enable_status_detection(HROBOT s, bool enable);
HRSDK_API int __stdcall eg_get_status_position(HROBOT s, int &type, bool &enable_detection, char *status, char *position, bool &is_connecting);
HRSDK_API int __stdcall eg_open_connection_v2(HROBOT s, int eg_id, int eg_type, int comm_type);
HRSDK_API int __stdcall eg_close_connection_v2(HROBOT s, int eg_id);
HRSDK_API int __stdcall eg_reset_v2(HROBOT s, int eg_id);
HRSDK_API int __stdcall eg_enable_status_detection_v2(HROBOT s, int eg_id, bool enable);
HRSDK_API int __stdcall eg_get_status_position_v2(HROBOT s, int eg_id, int &type, bool &enable_detection, char *status, char *position, bool &is_connecting);
HRSDK_API int __stdcall eg_grip_object_size(HROBOT s, int wp_id, double size);
HRSDK_API int __stdcall eg_grip_counter_index(HROBOT s, int index);
HRSDK_API int __stdcall eg_remove_object(HROBOT s, int index);

/*Set Password 5/21*/
HRSDK_API int __stdcall get_controller_mac(HROBOT s, char *mac);
HRSDK_API int __stdcall change_password(HROBOT s, char *old_password, char *new_password);
HRSDK_API int __stdcall reset_password(HROBOT s, char *key);

/* External Axis Setting*/
HRSDK_API int __stdcall get_ext_axis_setting(HROBOT s, int index, bool &enable, int &mode, double &high_limit, double &low_limit);
HRSDK_API int __stdcall set_ext_axis_setting(HROBOT s, int index, bool enable, int mode, double high_limit, double low_limit);
HRSDK_API int __stdcall get_ext_axis_setting_advanced(HROBOT s, int index, int &type, bool &math, bool &continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall set_ext_axis_setting_advanced(HROBOT s, int index, int type, bool math, bool continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall get_ext_axis_all_setting(HROBOT s, int index, bool &enable, int &mode, double &high_limit, double &low_limit, int &type, bool &math, bool &continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall set_ext_axis_all_setting(HROBOT s, int index, bool enable, int mode, double high_limit, double low_limit, int type, bool math, bool continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall ext_ptp(HROBOT s, int index, int direction, double position);
HRSDK_API int __stdcall ext_mastering(HROBOT s, int index);
HRSDK_API int __stdcall ext_ptp_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ext_ptp_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ext_lin_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall ext_lin_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall ext_asyptp(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall set_ext_driver_limit(HROBOT s, int index, bool enable, bool inverse, int negative_num, int positive_num);
HRSDK_API int __stdcall get_ext_driver_limit(HROBOT s, int index, bool &enable, bool &inverse, int &negative_num, int &positive_num, bool &N_light, bool &P_light);
HRSDK_API int __stdcall get_ext_encoder(HROBOT s, int32_t *EncCount);
HRSDK_API int __stdcall get_ext_calibration_tool_base(HROBOT s, int index, int &tool, int &base);
HRSDK_API int __stdcall get_ext_calibration_point(HROBOT s, int index, double point[3][7]);
HRSDK_API int __stdcall save_ext_calibration(HROBOT s, int index, int tool, int base, double point[3][7]);
HRSDK_API int __stdcall ext_linear_calibration(HROBOT s, int index, int step, int tool, int base, int &retrun_step);
HRSDK_API int __stdcall ext_rotary_calibration(HROBOT s, int index, int step, int tool, int base, int &retrun_step);
HRSDK_API int __stdcall get_ext_axis_group_parameter(HROBOT s, int index, int *param);
HRSDK_API int __stdcall set_ext_axis_group_parameter(HROBOT s, int index, int *param);
HRSDK_API int __stdcall get_ext_axis_calibration_base_coordinate(HROBOT s, int index, double base_coord[6]);

/* Home Warning Setting */
HRSDK_API int __stdcall get_home_warning_setting(HROBOT s, double *allow_error_value, double *allow_near_home);
HRSDK_API int __stdcall set_home_warning_setting(HROBOT s, double *allow_error_value, double *allow_near_home);

HRSDK_API int __stdcall get_robot_data(HROBOT s, int *sys_info, int *port_info, double *axis_info);
HRSDK_API int __stdcall set_ethernet_ip_config(HROBOT s, int channel, char *ip, int *config);
HRSDK_API int __stdcall get_ethernet_ip_config(HROBOT s, int channel, char *ip, int *config);
HRSDK_API int __stdcall ethernet_ip_connection(HROBOT s, int channel, bool connection);
HRSDK_API int __stdcall get_SIO_count(HROBOT s, int &count);
HRSDK_API int __stdcall get_fieldbus_rs_count(HROBOT s, int &count);
HRSDK_API int __stdcall import_comment(HROBOT s, char *path, int *import_idx);
HRSDK_API int __stdcall get_hrss_sdkver(HROBOT s, int &large_ver, int &small_ver, int &revision);
HRSDK_API int __stdcall get_hrsdk_sdkver(int &large_ver, int &small_ver, int &revision);
HRSDK_API int __stdcall get_robot_dh(HROBOT s, int type, double dh_value[][6]);
HRSDK_API int __stdcall get_gear_ratio(HROBOT s, double gear_ratio[6]);

/* Information2020/8/25 */
HRSDK_API int __stdcall get_version_information(HROBOT s, char *infor);

HRSDK_API int __stdcall get_DI_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DI_sim_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_DO_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_FI_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_FO_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_status_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_value_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_counter_value_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_counter_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_fieldbus_rs_srw_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_fieldbus_rs_srr_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_fieldbus_rs_parameter_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_fieldbus_rs_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_system_input_all(HROBOT s, int *value, wchar_t *comment);
HRSDK_API int __stdcall get_system_output_all(HROBOT s, int *value, wchar_t *comment);
HRSDK_API int __stdcall get_MI_config_all(HROBOT s, int *sim, int *value, int *type, int *start, int *end);
HRSDK_API int __stdcall get_MI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_MO_config_all(HROBOT s, int *value, int *type, int *start, int *end);
HRSDK_API int __stdcall get_MO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_PR_comment_array(HROBOT s, int *idx, int from_idx, int len, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_SI_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SI_sim_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SO_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_SO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_PR_array(HROBOT s, int *idx, int from_idx, int len, int *type, double *pos, int *tool, int *base, int &next_idx);
HRSDK_API int __stdcall get_RI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_RO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_RI_sim_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_RI_all(HROBOT s, int *values);
HRSDK_API int __stdcall get_RO_all(HROBOT s, int *values);
HRSDK_API int __stdcall get_VO_all(HROBOT s, int *values);

HRSDK_API int __stdcall set_DI_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_DI_sim_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_DO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_timer_value_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_counter_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_fieldbus_srw_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SI_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SI_sim_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_MO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_VO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_RO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall get_alarm_log_count(HROBOT s, int &size);
HRSDK_API int __stdcall get_alarm_log_msg(HROBOT s, int idx, wchar_t *msg);

/* Collision Detection */
HRSDK_API int __stdcall get_support_collision(HROBOT s, bool &is_supported);
HRSDK_API int __stdcall get_collision_detection_value(HROBOT s, int &value);
HRSDK_API int __stdcall set_collision_detection_value(HROBOT s, int value);
HRSDK_API int __stdcall get_collision_cold_start(HROBOT s, bool &enable);
HRSDK_API int __stdcall set_collision_cold_start(HROBOT s, bool enable);
HRSDK_API int __stdcall get_collision_output(HROBOT s, int &type, int &io_idx);
HRSDK_API int __stdcall set_collision_output(HROBOT s, int type, int io_idx);
HRSDK_API int __stdcall learn_collision_detection(HROBOT s, bool enable);
HRSDK_API int __stdcall save_collision_detection(HROBOT s);

/* Weld Setting */
HRSDK_API int __stdcall get_weld_touch_sensor_setting(HROBOT s, int &teach_mode, int &activate_signal, int &activate_signal_num, int &touch_signal, int &touch_signal_num, int &touch_logic, int &line_sensing);
HRSDK_API int __stdcall set_weld_touch_sensor_setting(HROBOT s, int teach_mode, int activate_signal, int activate_signal_num, int touch_signal, int touch_signal_num, int touch_logic, int line_sensing);
HRSDK_API int __stdcall get_weld_touch_sensor_schedule_parameter(HROBOT s, int schedule_index, double *reference_point, double *search_point, int &search_speed, int &return_speed, double &max_search_range, double &min_search_range, double &max_deviation);
HRSDK_API int __stdcall set_weld_touch_sensor_schedule_parameter(HROBOT s, int schedule_index, int search_speed, int return_speed, double max_search_range, double min_search_range, double max_deviation);
HRSDK_API int __stdcall get_weld_touch_sensor_schedule_parameter_limit(HROBOT s, int &min_search_speed, int &max_search_speed, int &min_return_speed, int &max_return_speed, double &min_max_search_range, double &max_max_search_range, double &min_min_search_range, double &max_min_search_range, double &min_deviation, double &max_deviation);
HRSDK_API int __stdcall get_weld_touch_sensor_schedule_comment(HROBOT s, int schedule_index, wchar_t *comment);
HRSDK_API int __stdcall set_weld_touch_sensor_schedule_comment(HROBOT s, int schedule_index, wchar_t *comment);
HRSDK_API int __stdcall set_weld_touch_sensor_reference_point(HROBOT s, int schedule_index, double *reference_point);
HRSDK_API int __stdcall set_weld_touch_sensor_search_point(HROBOT s, int schedule_index, double *search_point);
HRSDK_API int __stdcall get_weld_setting_parameter(HROBOT s, WeldSystemParameter *param);
HRSDK_API int __stdcall set_weld_setting_parameter(HROBOT s, WeldSystemParameter param);
HRSDK_API int __stdcall get_procedure_parameter(HROBOT s, int proc_id, WeldProcedureParameter *param);
HRSDK_API int __stdcall set_procedure_parameter(HROBOT s, int proc_id, WeldProcedureParameter param);
HRSDK_API int __stdcall get_procedure_comment(HROBOT s, int procedure_index, wchar_t *comment);
HRSDK_API int __stdcall set_procedure_comment(HROBOT s, int procedure_index, wchar_t *comment);
HRSDK_API int __stdcall get_schedule_parameter(HROBOT s, int proc_id, int sch_id, WeldScheduleParameter *param);
HRSDK_API int __stdcall set_schedule_parameter(HROBOT s, int proc_id, int sch_id, WeldScheduleParameter param);
HRSDK_API int __stdcall get_weave_parameter(HROBOT s, ROBOT_WEAVING_CONDITION *param);
HRSDK_API int __stdcall set_weave_parameter(HROBOT s, ROBOT_WEAVING_CONDITION param);
HRSDK_API int __stdcall get_weave_schedule_parameter(HROBOT s, int sch_id, ROBOT_WEAVING_CONDITION *param);
HRSDK_API int __stdcall set_weave_schedule_parameter(HROBOT s, int sch_id, ROBOT_WEAVING_CONDITION param);

/* TP03 */
HRSDK_API int __stdcall enable_rotary_encoder(HROBOT s, bool enable);
HRSDK_API int __stdcall set_rotary_encoder_type(HROBOT s, int num, int type);
HRSDK_API int __stdcall set_rotary_encoder_finetune(HROBOT s, double num);
HRSDK_API int __stdcall TP_send_msg(HROBOT s, unsigned char buf);

// HRSS4.0
HRSDK_API int __stdcall ptp(HROBOT s, double *pos_data, int tool, int base);

// HMI
HRSDK_API int __stdcall upload_hmi(HROBOT s, char *source_path, char *dest_path);
HRSDK_API int __stdcall close_hmi(HROBOT s);

extern uint16_t ts;
#ifdef __cplusplus
}
#endif
#endif  // HRSDK_HRSDK_EXPERT_H_
