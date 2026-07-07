1. 쓰기 가능 설정 파라미터 (odrv0.axis0.*)

odrv0.axis0.acim_estimator.config.slip_velocity  # float32 | [rad/s] | 유도전동기(ACIM) 슬립 속도 설정값
odrv0.axis0.config.calibration_lockin.accel  # float32 | [rad/s^2] | 캘리브레이션 락인(lock-in) 가속도
odrv0.axis0.config.calibration_lockin.current  # float32 | [A] | 캘리브레이션 락인 전류
odrv0.axis0.config.calibration_lockin.ramp_distance  # float32 | [rad] | 캘리브레이션 락인 램프 이동 거리
odrv0.axis0.config.calibration_lockin.ramp_time  # float32 | [s] | 캘리브레이션 락인 램프 소요 시간
odrv0.axis0.config.calibration_lockin.vel  # float32 | [rad/s] | 캘리브레이션 락인 목표 속도
odrv0.axis0.config.can.bus_vi_rate_ms  # uint32 | [ms] | CAN으로 버스 전압/전류 정보를 전송하는 주기
odrv0.axis0.config.can.controller_error_rate_ms  # uint32 | [ms] | CAN 컨트롤러 에러 메시지 전송 주기
odrv0.axis0.config.can.encoder_count_rate_ms  # uint32 | [ms] | CAN 엔코더 카운트 전송 주기
odrv0.axis0.config.can.encoder_error_rate_ms  # uint32 | [ms] | CAN 엔코더 에러 메시지 전송 주기
odrv0.axis0.config.can.encoder_rate_ms  # uint32 | [ms] | CAN 엔코더 추정값 전송 주기
odrv0.axis0.config.can.heartbeat_rate_ms  # uint32 | [ms] | CAN 하트비트 메시지 전송 주기
odrv0.axis0.config.can.iq_rate_ms  # uint32 | [ms] | CAN Iq(토크 전류) 전송 주기
odrv0.axis0.config.can.is_extended  # bool | CAN 확장 ID(29비트) 사용 여부
odrv0.axis0.config.can.motor_error_rate_ms  # uint32 | [ms] | CAN 모터 에러 메시지 전송 주기
odrv0.axis0.config.can.node_id  # uint32 | 이 축의 CAN 노드 ID
odrv0.axis0.config.can.sensorless_error_rate_ms  # uint32 | [ms] | CAN 센서리스 에러 메시지 전송 주기
odrv0.axis0.config.can.sensorless_rate_ms  # uint32 | [ms] | CAN 센서리스 추정값 전송 주기
odrv0.axis0.config.dir_gpio_pin  # uint16 | Step/Dir 방향(DIR) 입력 GPIO 핀 번호
odrv0.axis0.config.enable_sensorless_mode  # bool | 센서리스 모드 활성화 여부
odrv0.axis0.config.enable_step_dir  # bool | Step/Dir 입력 모드 활성화 여부
odrv0.axis0.config.enable_watchdog  # bool | 워치독 타이머 활성화 여부
odrv0.axis0.config.general_lockin.accel  # float32 | [rad/s^2] | 일반 락인(lock-in) 가속도
odrv0.axis0.config.general_lockin.current  # float32 | [A] | 일반 락인 전류
odrv0.axis0.config.general_lockin.finish_distance  # float32 | [rad] | 일반 락인 종료 거리
odrv0.axis0.config.general_lockin.finish_on_distance  # bool | 목표 거리 도달 시 락인 종료 여부
odrv0.axis0.config.general_lockin.finish_on_enc_idx  # bool | 엔코더 인덱스 검출 시 락인 종료 여부
odrv0.axis0.config.general_lockin.finish_on_vel  # bool | 목표 속도 도달 시 락인 종료 여부
odrv0.axis0.config.general_lockin.ramp_distance  # float32 | [rad] | 일반 락인 램프 이동 거리
odrv0.axis0.config.general_lockin.ramp_time  # float32 | [s] | 일반 락인 램프 소요 시간
odrv0.axis0.config.general_lockin.vel  # float32 | [rad/s] | 일반 락인 목표 속도
odrv0.axis0.config.sensorless_ramp.accel  # float32 | [rad/s^2] | 센서리스 기동 램프 가속도
odrv0.axis0.config.sensorless_ramp.current  # float32 | [A] | 센서리스 기동 램프 전류
odrv0.axis0.config.sensorless_ramp.finish_distance  # float32 | [rad] | 센서리스 기동 램프 종료 거리
odrv0.axis0.config.sensorless_ramp.finish_on_distance  # bool | 거리 도달 기준 센서리스 램프 종료 여부
odrv0.axis0.config.sensorless_ramp.finish_on_enc_idx  # bool | 엔코더 인덱스 검출 기준 센서리스 램프 종료 여부
odrv0.axis0.config.sensorless_ramp.finish_on_vel  # bool | 속도 도달 기준 센서리스 램프 종료 여부
odrv0.axis0.config.sensorless_ramp.ramp_distance  # float32 | [rad] | 센서리스 기동 램프 이동 거리
odrv0.axis0.config.sensorless_ramp.ramp_time  # float32 | [s] | 센서리스 기동 램프 소요 시간
odrv0.axis0.config.sensorless_ramp.vel  # float32 | [rad/s] | 센서리스 기동 램프 목표 속도
odrv0.axis0.config.startup_closed_loop_control  # bool | 부팅 시 폐루프 제어 자동 진입 여부
odrv0.axis0.config.startup_encoder_index_search  # bool | 부팅 시 엔코더 인덱스 서치 자동 실행 여부
odrv0.axis0.config.startup_encoder_offset_calibration  # bool | 부팅 시 엔코더 오프셋 캘리브레이션 자동 실행 여부
odrv0.axis0.config.startup_homing  # bool | 부팅 시 호밍 동작 자동 실행 여부
odrv0.axis0.config.startup_motor_calibration  # bool | 부팅 시 모터 캘리브레이션 자동 실행 여부
odrv0.axis0.config.step_dir_always_on  # bool | 폐루프가 아닐 때도 Step/Dir 입력을 항상 활성화할지 여부
odrv0.axis0.config.step_gpio_pin  # uint16 | Step/Dir STEP 입력 GPIO 핀 번호
odrv0.axis0.config.watchdog_timeout  # float32 | [s] | 워치독 타임아웃 시간
odrv0.axis0.controller.anticogging_valid  # bool | 안티코깅(코깅 토크 보상) 맵 유효 여부
odrv0.axis0.controller.autotuning.frequency  # float32 | [Hz] | 오토튜닝 가진(excitation) 주파수
odrv0.axis0.controller.autotuning.pos_amplitude  # float32 | [turns] | 오토튜닝 위치 가진 진폭
odrv0.axis0.controller.autotuning.torque_amplitude  # float32 | [N·m] | 오토튜닝 토크 가진 진폭
odrv0.axis0.controller.autotuning.vel_amplitude  # float32 | [turns/sec] | 오토튜닝 속도 가진 진폭
odrv0.axis0.controller.autotuning_phase  # float32 | [rad] | 오토튜닝 진행 위상값
odrv0.axis0.controller.config.anticogging.anticogging_enabled  # bool | 안티코깅 보상 적용 사용 여부
odrv0.axis0.controller.config.anticogging.calib_pos_threshold  # float32 | 안티코깅 캘리브레이션 위치 오차 허용 임계값
odrv0.axis0.controller.config.anticogging.calib_vel_threshold  # float32 | 안티코깅 캘리브레이션 속도 오차 허용 임계값
odrv0.axis0.controller.config.anticogging.pre_calibrated  # bool | 안티코깅 사전 캘리브레이션 완료 여부
odrv0.axis0.controller.config.axis_to_mirror  # uint8 | 미러링(동기 추종) 대상 축 번호
odrv0.axis0.controller.config.circular_setpoint_range  # float32 | 원형(순환) 위치 설정값 범위
odrv0.axis0.controller.config.circular_setpoints  # bool | 위치 설정값 순환(원형) 모드 사용 여부
odrv0.axis0.controller.config.control_mode  # enum:ControlMode | 제어 모드(전압/전류·토크/속도/위치)
odrv0.axis0.controller.config.electrical_power_bandwidth  # float32 | [rad/s] | 전기 파워 추정 필터 대역폭
odrv0.axis0.controller.config.enable_gain_scheduling  # bool | 저속 구간 게인 스케줄링 사용 여부
odrv0.axis0.controller.config.enable_overspeed_error  # bool | 과속 에러 감지 사용 여부
odrv0.axis0.controller.config.enable_torque_mode_vel_limit  # bool | 토크 모드에서 속도 제한 적용 여부
odrv0.axis0.controller.config.enable_vel_limit  # bool | 속도 제한 적용 여부
odrv0.axis0.controller.config.gain_scheduling_width  # float32 | 게인 스케줄링 적용 구간 폭
odrv0.axis0.controller.config.homing_speed  # float32 | [turn/s] | 호밍 동작 속도
odrv0.axis0.controller.config.inertia  # float32 | [N·m/(turn/s^2)] | 관성 보상 계수
odrv0.axis0.controller.config.input_filter_bandwidth  # float32 | [rad/s] | INPUT_MODE_POS_FILTER 모드에서 사용하는 입력 필터 대역폭
odrv0.axis0.controller.config.input_mode  # enum:InputMode | 입력값 처리 모드(패스스루/필터/램프/트랩궤적 등)
odrv0.axis0.controller.config.load_encoder_axis  # uint8 | 부하측 엔코더로 사용할 축 번호
odrv0.axis0.controller.config.mechanical_power_bandwidth  # float32 | [rad/s] | 기계 파워 추정 필터 대역폭
odrv0.axis0.controller.config.mirror_ratio  # float32 | 미러링 시 위치 비율
odrv0.axis0.controller.config.pos_gain  # float32 | [(turn/s)/turn] | 위치 제어 비례 게인
odrv0.axis0.controller.config.spinout_electrical_power_threshold  # float32 | [Watt] | 스핀아웃(비정상 공회전) 감지용 전기 파워 임계값
odrv0.axis0.controller.config.spinout_mechanical_power_threshold  # float32 | [Watt] | 스핀아웃 감지용 기계 파워 임계값
odrv0.axis0.controller.config.steps_per_circular_range  # int32 | 순환 범위 1주기당 스텝 수
odrv0.axis0.controller.config.torque_mirror_ratio  # float32 | 미러링 시 토크 비율
odrv0.axis0.controller.config.torque_ramp_rate  # float32 | [N·m/sec] | 토크 변화 램프 속도
odrv0.axis0.controller.config.vel_gain  # float32 | [N·m/(turn/s)] | 속도 제어 비례 게인(P)
odrv0.axis0.controller.config.vel_integrator_gain  # float32 | [N·m/(turn/s * s)] | 속도 제어 적분 게인(I)
odrv0.axis0.controller.config.vel_integrator_limit  # float32 | [N·m] | 속도 적분기 출력 제한값
odrv0.axis0.controller.config.vel_limit  # float32 | [turn/s] | 속도 제한값
odrv0.axis0.controller.config.vel_limit_tolerance  # float32 | 속도 제한 초과 허용 배수
odrv0.axis0.controller.config.vel_ramp_rate  # float32 | [turn/s^2] | 속도 변화 램프 속도
odrv0.axis0.controller.input_pos  # float32 | [turn] | 위치 제어 목표 입력값
odrv0.axis0.controller.input_torque  # float32 | [N·m] | 토크 제어 목표 입력값
odrv0.axis0.controller.input_vel  # float32 | [turn/s] | 속도 제어 목표 입력값
odrv0.axis0.controller.last_error_time  # float32 | 컨트롤러 마지막 에러 발생 시각
odrv0.axis0.controller.vel_integrator_torque  # float32 | [N·m] | 속도 적분기의 누적 토크 출력값
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin  # uint16 | 앱솔루트 SPI 엔코더 CS(칩 셀렉트) GPIO 핀 번호
odrv0.axis0.encoder.config.bandwidth  # float32 | [rad/s] | 엔코더 위치/속도 추정 필터 대역폭
odrv0.axis0.encoder.config.calib_range  # float32 | [turn] | 엔코더 캘리브레이션 허용 오차 범위
odrv0.axis0.encoder.config.calib_scan_distance  # float32 | [rad (electrical)] | 엔코더 캘리브레이션 스캔 이동 거리(전기각)
odrv0.axis0.encoder.config.calib_scan_omega  # float32 | [rad/s (electrical)] | 엔코더 캘리브레이션 스캔 각속도(전기각)
odrv0.axis0.encoder.config.cpr  # int32 | 엔코더 1회전당 카운트 수(CPR)
odrv0.axis0.encoder.config.direction  # int32 | 엔코더 회전 방향(캘리브레이션으로 결정됨)
odrv0.axis0.encoder.config.enable_phase_interpolation  # bool | 엔코더 위상 보간(interpolation) 사용 여부
odrv0.axis0.encoder.config.find_idx_on_lockin_only  # bool | 락인 모드에서만 인덱스 펄스 탐색 여부
odrv0.axis0.encoder.config.hall_polarity  # uint8 | 홀센서 극성 설정값
odrv0.axis0.encoder.config.hall_polarity_calibrated  # bool | 홀센서 극성 캘리브레이션 완료 여부
odrv0.axis0.encoder.config.ignore_illegal_hall_state  # bool | 비정상 홀 상태 무시 여부
odrv0.axis0.encoder.config.index_offset  # float32 | 엔코더 인덱스 위치 오프셋
odrv0.axis0.encoder.config.mode  # enum:Mode | 엔코더 종류/모드(증분식/홀센서/SPI 등)
odrv0.axis0.encoder.config.phase_offset  # int32 | 엔코더 위상 오프셋(정수, 카운트 단위)
odrv0.axis0.encoder.config.phase_offset_float  # float32 | 엔코더 위상 오프셋(실수값)
odrv0.axis0.encoder.config.pre_calibrated  # bool | 엔코더 사전 캘리브레이션 완료 여부
odrv0.axis0.encoder.config.sincos_gpio_pin_cos  # uint16 | Sin/Cos 엔코더 Cos 입력 GPIO 핀 번호
odrv0.axis0.encoder.config.sincos_gpio_pin_sin  # uint16 | Sin/Cos 엔코더 Sin 입력 GPIO 핀 번호
odrv0.axis0.encoder.config.use_index  # bool | 인덱스 펄스 사용 여부
odrv0.axis0.encoder.config.use_index_offset  # bool | 인덱스 오프셋 적용 여부
odrv0.axis0.encoder.pos_abs  # int32 | 앱솔루트 엔코더 절대 위치값
odrv0.axis0.is_homed  # bool | 축이 호밍(원점 복귀)을 성공적으로 완료했는지 여부
odrv0.axis0.max_endstop.config.debounce_ms  # uint32 | [ms] | 최대 리미트 스위치 디바운스 시간
odrv0.axis0.max_endstop.config.enabled  # bool | 최대 리미트 스위치 사용 여부
odrv0.axis0.max_endstop.config.gpio_num  # uint16 | 최대 리미트 스위치 입력 GPIO 핀 번호
odrv0.axis0.max_endstop.config.is_active_high  # bool | 최대 리미트 스위치 액티브 하이 여부
odrv0.axis0.max_endstop.config.offset  # float32 | [turns] | 최대 리미트 위치 오프셋
odrv0.axis0.mechanical_brake.config.gpio_num  # uint16 | 기계식 브레이크 제어 GPIO 핀 번호
odrv0.axis0.mechanical_brake.config.is_active_low  # bool | 기계식 브레이크 액티브 로우 여부
odrv0.axis0.min_endstop.config.debounce_ms  # uint32 | [ms] | 최소 리미트 스위치 디바운스 시간
odrv0.axis0.min_endstop.config.enabled  # bool | 최소 리미트 스위치 사용 여부
odrv0.axis0.min_endstop.config.gpio_num  # uint16 | 최소 리미트 스위치 입력 GPIO 핀 번호
odrv0.axis0.min_endstop.config.is_active_high  # bool | 최소 리미트 스위치 액티브 하이 여부
odrv0.axis0.min_endstop.config.offset  # float32 | [turns] | 최소 리미트 위치 오프셋
odrv0.axis0.motor.DC_calib_phA  # float32 | A상 DC 오프셋 캘리브레이션 값
odrv0.axis0.motor.DC_calib_phB  # float32 | B상 DC 오프셋 캘리브레이션 값
odrv0.axis0.motor.DC_calib_phC  # float32 | C상 DC 오프셋 캘리브레이션 값
odrv0.axis0.motor.config.I_bus_hard_max  # float32 | [A] | 버스 전류 상한 하드 리미트
odrv0.axis0.motor.config.I_bus_hard_min  # float32 | [A] | 버스 전류 하한(회생 방향) 하드 리미트
odrv0.axis0.motor.config.I_leak_max  # float32 | [A] | 누설 전류 최대 허용치
odrv0.axis0.motor.config.R_wL_FF_enable  # bool | 저항/인덕턴스 피드포워드 보상 사용 여부
odrv0.axis0.motor.config.acim_autoflux_attack_gain  # float32 | ACIM 오토플럭스 상승(attack) 게인
odrv0.axis0.motor.config.acim_autoflux_decay_gain  # float32 | ACIM 오토플럭스 감쇠(decay) 게인
odrv0.axis0.motor.config.acim_autoflux_enable  # bool | ACIM 오토플럭스 제어 사용 여부
odrv0.axis0.motor.config.acim_autoflux_min_Id  # float32 | ACIM 오토플럭스 최소 여자 전류(Id)
odrv0.axis0.motor.config.acim_gain_min_flux  # float32 | ACIM 최소 자속 기준 게인값
odrv0.axis0.motor.config.bEMF_FF_enable  # bool | 역기전력(BEMF) 피드포워드 보상 사용 여부
odrv0.axis0.motor.config.calibration_current  # float32 | 모터 캘리브레이션 시 사용 전류
odrv0.axis0.motor.config.current_control_bandwidth  # float32 | [rad/s] | 전류 제어 루프 대역폭
odrv0.axis0.motor.config.current_lim  # float32 | [A] | 모터 전류 제한값
odrv0.axis0.motor.config.current_lim_margin  # float32 | [A] | 전류 제한 여유 마진
odrv0.axis0.motor.config.dc_calib_tau  # float32 | DC 오프셋 캘리브레이션 시정수
odrv0.axis0.motor.config.inverter_temp_limit_lower  # float32 | [°C] | 인버터(FET) 온도 제한 하한(출력 저감 시작 온도)
odrv0.axis0.motor.config.inverter_temp_limit_upper  # float32 | [°C] | 인버터(FET) 온도 제한 상한(완전 차단 온도)
odrv0.axis0.motor.config.motor_type  # enum:MotorType | 모터 타입(하이커런트/짐벌/ACIM)
odrv0.axis0.motor.config.phase_inductance  # float32 | [henry] | 모터 상 인덕턴스(캘리브레이션 결과)
odrv0.axis0.motor.config.phase_resistance  # float32 | [ohm] | 모터 상 저항(캘리브레이션 결과)
odrv0.axis0.motor.config.pole_pairs  # int32 | 모터 극쌍수(pole pairs)
odrv0.axis0.motor.config.pre_calibrated  # bool | 모터 사전 캘리브레이션 완료 여부
odrv0.axis0.motor.config.requested_current_range  # float32 | [A] | 요청 전류 측정 범위
odrv0.axis0.motor.config.resistance_calib_max_voltage  # float32 | 저항 캘리브레이션 시 최대 인가 전압
odrv0.axis0.motor.config.torque_constant  # float32 | [N·m/A] | 토크 상수(Kt)
odrv0.axis0.motor.config.torque_lim  # float32 | [N·m] | 토크 제한값
odrv0.axis0.motor.current_control.I_measured_report_filter_k  # float32 | 측정 전류 보고용 저역통과 필터 계수
odrv0.axis0.motor.current_control.v_current_control_integral_d  # float32 | 전류 제어 D축 적분항 누적값
odrv0.axis0.motor.current_control.v_current_control_integral_q  # float32 | 전류 제어 Q축 적분항 누적값
odrv0.axis0.motor.fet_thermistor.config.enabled  # bool | FET 온도 센서 사용 여부
odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower  # float32 | FET 온도 제한 하한(출력 저감 시작 온도)
odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper  # float32 | FET 온도 제한 상한(완전 차단 온도)
odrv0.axis0.motor.last_error_time  # float32 | 모터 마지막 에러 발생 시각
odrv0.axis0.motor.motor_thermistor.config.enabled  # bool | 모터 온도 센서 사용 여부
odrv0.axis0.motor.motor_thermistor.config.gpio_pin  # uint16 | 모터 온도 센서 입력 GPIO 핀 번호
odrv0.axis0.motor.motor_thermistor.config.poly_coefficient_0  # float32 | 모터 온도 변환식 다항식 계수(0차항)
odrv0.axis0.motor.motor_thermistor.config.poly_coefficient_1  # float32 | 모터 온도 변환식 다항식 계수(1차항)
odrv0.axis0.motor.motor_thermistor.config.poly_coefficient_2  # float32 | 모터 온도 변환식 다항식 계수(2차항)
odrv0.axis0.motor.motor_thermistor.config.poly_coefficient_3  # float32 | 모터 온도 변환식 다항식 계수(3차항)
odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower  # float32 | 모터 온도 제한 하한(출력 저감 시작 온도)
odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper  # float32 | 모터 온도 제한 상한(완전 차단 온도)
odrv0.axis0.motor.phase_current_rev_gain  # float32 | 상전류 측정 리버스 게인(부호/스케일 보정값)
odrv0.axis0.requested_state  # enum:AxisState | 사용자가 요청(명령)한 축 상태값
odrv0.axis0.sensorless_estimator.config.observer_gain  # float32 | 센서리스 관측기(옵저버) 게인
odrv0.axis0.sensorless_estimator.config.pll_bandwidth  # float32 | 센서리스 PLL(위상고정루프) 대역폭
odrv0.axis0.sensorless_estimator.config.pm_flux_linkage  # float32 | 영구자석 자속 쇄교수(모터 설계값)
odrv0.axis0.task_times.acim_estimator_update.max_length  # uint32 | ACIM 추정기 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.can_heartbeat.max_length  # uint32 | CAN 하트비트 태스크 최대 실행 시간
odrv0.axis0.task_times.controller_update.max_length  # uint32 | 컨트롤러 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.current_controller_update.max_length  # uint32 | 전류 제어 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.current_sense.max_length  # uint32 | 전류 센싱 태스크 최대 실행 시간
odrv0.axis0.task_times.dc_calib.max_length  # uint32 | DC 오프셋 캘리브레이션 태스크 최대 실행 시간
odrv0.axis0.task_times.encoder_update.max_length  # uint32 | 엔코더 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.endstop_update.max_length  # uint32 | 리미트 스위치 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.motor_update.max_length  # uint32 | 모터 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.open_loop_controller_update.max_length  # uint32 | 오픈루프 제어 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.pwm_update.max_length  # uint32 | PWM 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.sensorless_estimator_update.max_length  # uint32 | 센서리스 추정기 업데이트 태스크 최대 실행 시간
odrv0.axis0.task_times.thermistor_update.max_length  # uint32 | 온도 센서 업데이트 태스크 최대 실행 시간
odrv0.axis0.trap_traj.config.accel_limit  # float32 | [turn/s^2] | 트랩(사다리꼴) 궤적 가속도 제한값
odrv0.axis0.trap_traj.config.decel_limit  # float32 | [turn/s^2] | 트랩(사다리꼴) 궤적 감속도 제한값
odrv0.axis0.trap_traj.config.vel_limit  # float32 | [turn/s] | 트랩(사다리꼴) 궤적 속도 제한값



2. 쓰기 가능 설정 파라미터 (odrv0.can., odrv0.config., 기타)\

odrv0.can.config.baud_rate  # uint32 | can 통신 속도 보드레이트 설정
odrv0.can.config.protocol  # enum:Protocol | CAN 프로토콜 종류 설정(Simple 등)
odrv0.config.brake_resistance  # float32 | [ohm] | ODrive에 연결된 브레이크 저항값
odrv0.config.dc_bus_overvoltage_ramp_end  # float32 | 과전압 램프(회생 제동 제어) 종료 전압. enable_dc_bus_overvoltage_ramp 참고
odrv0.config.dc_bus_overvoltage_ramp_start  # float32 | 과전압 램프(회생 제동 제어) 시작 전압. enable_dc_bus_overvoltage_ramp 참고
odrv0.config.dc_bus_overvoltage_trip_level  # float32 | [V] | 이 전압을 초과하면 모터 동작을 정지시키는 과전압 트립 레벨
odrv0.config.dc_bus_undervoltage_trip_level  # float32 | [V] | 이 전압보다 낮아지면 모터 동작을 정지시키는 저전압 트립 레벨
odrv0.config.dc_max_negative_current  # float32 | [A] | 전원/PWM/아날로그 등)
odrv0.config.gpio11_mode  # enum:GpioMode | GPIO11 핀 동작 모드
odrv0.config.gpio12_mode  # enum:GpioMode | GPIO12 핀 동작 모드
odrv0.config.gpio13_mode  # enum:GpioMode | GPIO13 핀 동작 모드
odrv0.config.gpio14_mode  # enum:GpioMode | GPIO14 핀 동작 모드
odrv0.config.gpio15_mode  # enum:GpioMode | GPIO15 핀 동작 모드
odrv0.config.gpio16_mode  # enum:GpioMode | GPIO16 핀 동작 모드
odrv0.config.gpio1_mode  # enum:GpioMode | GPIO1 핀 동작 모드
odrv0.config.gpio1_pwm_mapping.endpoint  # endpoint_ref | GPIO1 PWM 입 공급 장치가 흡수(회생 싱크)할 수 있는 최대 전류
odrv0.config.dc_max_positive_current  # float32 | [A] | 전원 공급 장치가 공급할 수 있는 최대 전류
odrv0.config.enable_brake_resistor  # bool | 브레이크 저항 사용 여부
odrv0.config.enable_can_a  # bool | CAN A 포트 사용 여부
odrv0.config.enable_dc_bus_overvoltage_ramp  # bool | DC 버스 과전압 램프(회생 제동 제어) 기능 사용 여부
odrv0.config.enable_i2c_a  # bool | I2C A 포트 사용 여부
odrv0.config.enable_uart_a  # bool | UART_A 포트 사용 여부
odrv0.config.enable_uart_b  # bool | UART_B 포트 사용 여부
odrv0.config.enable_uart_c  # bool | UART_C 포트 사용 여부
odrv0.config.error_gpio_pin  # uint32 | 에러 표시 출력 GPIO 핀 번호
odrv0.config.gpio10_mode  # enum:GpioMode | GPIO10 핀 동작 모드(디지털력값을 매핑할 대상 파라미터
odrv0.config.gpio1_pwm_mapping.max  # float32 | GPIO1 PWM 매핑 시 대상 파라미터 최대값
odrv0.config.gpio1_pwm_mapping.min  # float32 | GPIO1 PWM 매핑 시 대상 파라미터 최소값
odrv0.config.gpio2_mode  # enum:GpioMode | GPIO2 핀 동작 모드
odrv0.config.gpio2_pwm_mapping.endpoint  # endpoint_ref | GPIO2 PWM 입력값을 매핑할 대상 파라미터
odrv0.config.gpio2_pwm_mapping.max  # float32 | GPIO2 PWM 매핑 시 대상 파라미터 최대값
odrv0.config.gpio2_pwm_mapping.min  # float32 | GPIO2 PWM 매핑 시 대상 파라미터 최소값
odrv0.config.gpio3_analog_mapping.endpoint  # endpoint_ref | GPIO3 아날로그 입력값을 매핑할 대상 파라미터
odrv0.config.gpio3_analog_mapping.max  # float32 | GPIO3 아날로그 매핑 시 대상 파라미터 최대값
odrv0.config.gpio3_analog_mapping.min  # float32 | GPIO3 아날로그 매핑 시 대상 파라미터 최소값
odrv0.config.gpio3_mode  # enum:GpioMode | GPIO3 핀 동작 모드
odrv0.config.gpio3_pwm_mapping.endpoint  # endpoint_ref | GPIO3 PWM 입력값을 매핑할 대상 파라미터
odrv0.config.gpio3_pwm_mapping.max  # float32 | GPIO3 PWM 매핑 시 대상 파라미터 최대값
odrv0.config.gpio3_pwm_mapping.min  # float32 | GPIO3 PWM 매핑 시 대상 파라미터 최소값
odrv0.config.gpio4_analog_mapping.endpoint  # endpoint_ref | GPIO4 아날로그 입력값을 매핑할 대상 파라미터
odrv0.config.gpio4_analog_mapping.max  # float32 | GPIO4 아날로그 매핑 시 대상 파라미터 최대값
odrv0.config.gpio4_analog_mapping.min  # float32 | GPIO4 아날로그 매핑 시 대상 파라미터 최소값
odrv0.config.gpio4_mode  # enum:GpioMode | GPIO4 핀 동작 모드
odrv0.config.gpio4_pwm_mapping.endpoint  # endpoint_ref | GPIO4 PWM 입력값을 매핑할 대상 파라미터
odrv0.config.gpio4_pwm_mapping.max  # float32 | GPIO4 PWM 매핑 시 대상 파라미터 최대값
odrv0.config.gpio4_pwm_mapping.min  # float32 | GPIO4 PWM 매핑 시 대상 파라미터 최소값
odrv0.config.gpio5_mode  # enum:GpioMode | GPIO5 핀 동작 모드
odrv0.config.gpio6_mode  # enum:GpioMode | GPIO6 핀 동작 모드
odrv0.config.gpio7_mode  # enum:GpioMode | GPIO7 핀 동작 모드
odrv0.config.gpio8_mode  # enum:GpioMode | GPIO8 핀 동작 모드
odrv0.config.gpio9_mode  # enum:GpioMode | GPIO9 핀 동작 모드
odrv0.config.max_regen_current  # float32 | [Amps] | 최대 회생(regen) 전류 제한값
odrv0.config.uart0_protocol  # enum:StreamProtocolType | UART0 포트 프로토콜 종류
odrv0.config.uart1_protocol  # enum:StreamProtocolType | UART1 포트 프로토콜 종류
odrv0.config.uart2_protocol  # enum:StreamProtocolType | UART2 포트 프로토콜 종류
odrv0.config.uart_a_baudrate  # uint32 | [baud/s] | UART_A 인터페이스 통신 속도(보드레이트)
odrv0.config.uart_b_baudrate  # uint32 | [baud/s] | UART_B 인터페이스 통신 속도(보드레이트)
odrv0.config.uart_c_baudrate  # uint32 | UART_C 인터페이스 통신 속도(보드레이트)
odrv0.config.usb_cdc_protocol  # enum:StreamProtocolType | USB CDC(가상 시리얼) 포트 프로토콜 종류
odrv0.ibus_report_filter_k  # float32 | 버스 전류 보고값 저역통과 필터 계수
odrv0.task_timers_armed  # bool | 태스크 실행 시간 측정(타이머) 활성화 여부
odrv0.task_times.control_loop_checks.max_length  # uint32 | 제어 루프 점검 태스크 최대 실행 시간
odrv0.task_times.control_loop_misc.max_length  # uint32 | 제어 루프 기타 처리 태스크 최대 실행 시간
odrv0.task_times.dc_calib_wait.max_length  # uint32 | DC 캘리브레이션 대기 태스크 최대 실행 시간
odrv0.task_times.sampling.max_length  # uint32 | ADC 샘플링 태스크 최대 실행 시간
odrv0.test_property  # uint32 | 테스트/디버그용 임시 속성값


3. 함수 (설정 저장/캘리브레이션 등)

odrv0.axis0.controller.get_anticogging_value(index)  # 지정 인덱스의 안티코깅 보상값을 반환
odrv0.axis0.controller.move_incremental(displacement, from_input_pos)  # 현재/입력 위치 기준으로 상대 이동 명령 실행
odrv0.axis0.controller.remove_anticogging_bias()  # 안티코깅 맵에서 평균 바이어스 성분을 제거
odrv0.axis0.controller.start_anticogging_calibration()  # 안티코깅 캘리브레이션 시작
odrv0.axis0.encoder.set_linear_count(count)  # 엔코더 리니어 카운트 값을 지정값으로 강제 설정
odrv0.axis0.mechanical_brake.engage()  # 기계식 브레이크 체결(작동)
odrv0.axis0.mechanical_brake.release()  # 기계식 브레이크 해제
odrv0.axis0.watchdog_feed()  # 워치독 타이머 리셋(피드)
odrv0.clear_errors()  # 모든 에러 플래그 초기화
odrv0.enter_dfu_mode()  # DFU(펌웨어 업데이트) 모드 진입
odrv0.erase_configuration()  # 저장된 설정을 초기화(공장 초기화)
odrv0.get_adc_voltage(gpio)  # 지정 GPIO 핀의 ADC 측정 전압값 반환
odrv0.get_dma_status(stream_num)  # 지정 DMA 스트림의 상태 반환
odrv0.get_drv_fault()  # 게이트 드라이버(DRV) 하드웨어 고장 코드 반환
odrv0.get_gpio_states()  # 전체 GPIO 핀들의 현재 상태 비트마스크 반환
odrv0.get_interrupt_status(irqn)  # 지정 인터럽트 번호의 발생 횟수 반환
odrv0.oscilloscope.get_val(index)  # 오실로스코프 버퍼의 지정 인덱스 값 반환
odrv0.reboot()  # 보드 재부팅
odrv0.save_configuration()  # 현재 설정을 비휘발성 메모리에 저장
odrv0.test_function(delta)  # 테스트/디버그용 함수 실행
dump_errors(odrv0)  # odrivetool 내장 함수 - 전체 에러 출력

4. 읽기 전용 값 (상태/측정값 - 설정 불가)

odrv0.axis0.acim_estimator.phase_offset  # float32 | [rad] | ACIM 추정기 위상 오프셋 측정값
odrv0.axis0.acim_estimator.rotor_flux  # float32 | [A] | ACIM 로터 자속 추정값
odrv0.axis0.acim_estimator.slip_vel  # float32 | [rad/s] | ACIM 슬립 속도 추정값
odrv0.axis0.acim_estimator.stator_phase  # float32 | [rad] | ACIM 고정자 위상각 추정값
odrv0.axis0.acim_estimator.stator_phase_vel  # float32 | [rad/s] | ACIM 고정자 위상 각속도 추정값
odrv0.axis0.controller.config.anticogging.calib_anticogging  # bool | 안티코깅 캘리브레이션 진행 중 여부
odrv0.axis0.controller.config.anticogging.cogging_ratio  # float32 | 코깅 토크 비율 측정값
odrv0.axis0.controller.config.anticogging.index  # uint32 | 안티코깅 캘리브레이션 현재 진행 인덱스
odrv0.axis0.controller.electrical_power  # float32 | [Watt] | 전기적 파워 추정값
odrv0.axis0.controller.error  # bitmask | 컨트롤러 에러 코드
odrv0.axis0.controller.mechanical_power  # float32 | [Watt] | 기계적 파워 추정값
odrv0.axis0.controller.pos_setpoint  # float32 | [turn] | 현재 위치 설정값(내부 계산 결과)
odrv0.axis0.controller.torque_setpoint  # float32 | [N·m] | 현재 토크 설정값(내부 계산 결과)
odrv0.axis0.controller.trajectory_done  # bool | 궤적(trajectory) 이동 완료 여부
odrv0.axis0.controller.vel_setpoint  # float32 | [turn/s] | 현재 속도 설정값(내부 계산 결과)
odrv0.axis0.current_state  # enum:AxisState | 축의 현재 상태값
odrv0.axis0.encoder.calib_scan_response  # float32 | 엔코더 캘리브레이션 스캔 응답값
odrv0.axis0.encoder.count_in_cpr  # int32 | [counts] | CPR 범위 내 엔코더 카운트값
odrv0.axis0.encoder.delta_pos_cpr_counts  # float32 | [counts] | 직전 주기 대비 위치 변화량(CPR 카운트 단위)
odrv0.axis0.encoder.error  # bitmask | 엔코더 에러 코드
odrv0.axis0.encoder.hall_state  # uint8 | 홀센서 현재 상태값
odrv0.axis0.encoder.index_found  # bool | 인덱스 펄스 검출 여부
odrv0.axis0.encoder.interpolation  # float32 | 엔코더 위상 보간값
odrv0.axis0.encoder.is_ready  # bool | 엔코더 준비 완료 여부
odrv0.axis0.encoder.phase  # float32 | 엔코더 전기적 위상각
odrv0.axis0.encoder.pos_circular  # float32 | [turns] | 순환 범위로 감싼 위치값
odrv0.axis0.encoder.pos_cpr_counts  # float32 | [counts] | CPR 기준 위치 카운트값
odrv0.axis0.encoder.pos_estimate  # float32 | [turns] | 추정된 위치값(회전수 단위)
odrv0.axis0.encoder.pos_estimate_counts  # float32 | [counts] | 추정된 위치값(카운트 단위)
odrv0.axis0.encoder.shadow_count  # int32 | [counts] | 엔코더 섀도우 카운트(원시 누적값)
odrv0.axis0.encoder.spi_error_rate  # float32 | SPI 엔코더 통신 에러 발생률
odrv0.axis0.encoder.vel_estimate  # float32 | [turn/s] | 추정된 속도값(회전수/초)
odrv0.axis0.encoder.vel_estimate_counts  # float32 | [counts/sec] | 추정된 속도값(카운트/초)
odrv0.axis0.error  # bitmask | 축 전체 에러 코드
odrv0.axis0.last_drv_fault  # uint32 | 마지막 게이트 드라이버 고장 코드
odrv0.axis0.max_endstop.endstop_state  # bool | 최대 리미트 스위치 현재 상태
odrv0.axis0.min_endstop.endstop_state  # bool | 최소 리미트 스위치 현재 상태
odrv0.axis0.motor.I_bus  # float32 | [A] | 모터가 소비 중인 버스 전류값
odrv0.axis0.motor.current_control.Ialpha_measured  # float32 | 알파(α)축 측정 전류
odrv0.axis0.motor.current_control.Ibeta_measured  # float32 | 베타(β)축 측정 전류
odrv0.axis0.motor.current_control.Id_measured  # float32 | [A] | D축(자속 성분) 측정 전류
odrv0.axis0.motor.current_control.Id_setpoint  # float32 | D축 전류 설정값
odrv0.axis0.motor.current_control.Iq_measured  # float32 | [A] | Q축(토크 성분) 측정 전류
odrv0.axis0.motor.current_control.Iq_setpoint  # float32 | Q축 전류 설정값
odrv0.axis0.motor.current_control.Vd_setpoint  # float32 | D축 전압 설정값
odrv0.axis0.motor.current_control.Vq_setpoint  # float32 | Q축 전압 설정값
odrv0.axis0.motor.current_control.final_v_alpha  # float32 | 최종 알파축 출력 전압
odrv0.axis0.motor.current_control.final_v_beta  # float32 | 최종 베타축 출력 전압
odrv0.axis0.motor.current_control.i_gain  # float32 | 전류 제어 적분 게인(내부 계산값)
odrv0.axis0.motor.current_control.p_gain  # float32 | 전류 제어 비례 게인(내부 계산값)
odrv0.axis0.motor.current_control.phase  # float32 | 전류 제어에 사용되는 전기적 위상각
odrv0.axis0.motor.current_control.phase_vel  # float32 | 전류 제어에 사용되는 위상 각속도
odrv0.axis0.motor.current_control.power  # float32 | [W] | 전류 제어단 계산 파워값
odrv0.axis0.motor.current_meas_phA  # float32 | A상 측정 전류값
odrv0.axis0.motor.current_meas_phB  # float32 | B상 측정 전류값
odrv0.axis0.motor.current_meas_phC  # float32 | C상 측정 전류값
odrv0.axis0.motor.effective_current_lim  # float32 | [A] | 현재 적용 중인 유효 전류 제한값
odrv0.axis0.motor.error  # bitmask | 모터 에러 코드
odrv0.axis0.motor.fet_thermistor.temperature  # float32 | [°C] | FET 온도 측정값
odrv0.axis0.motor.is_armed  # bool | 모터 암(구동 활성화) 상태 여부
odrv0.axis0.motor.is_calibrated  # bool | 모터 캘리브레이션 완료 여부
odrv0.axis0.motor.max_allowed_current  # float32 | [A] | 하드웨어 기준 최대 허용 전류
odrv0.axis0.motor.max_dc_calib  # float32 | [A] | DC 오프셋 캘리브레이션 최대 허용치
odrv0.axis0.motor.motor_thermistor.temperature  # float32 | [°C] | 모터 온도 측정값
odrv0.axis0.motor.n_evt_current_measurement  # uint32 | 전류 측정 인터럽트 발생 횟수
odrv0.axis0.motor.n_evt_pwm_update  # uint32 | PWM 업데이트 인터럽트 발생 횟수
odrv0.axis0.sensorless_estimator.error  # bitmask | 센서리스 추정기 에러 코드
odrv0.axis0.sensorless_estimator.phase  # float32 | [rad] | 센서리스 추정 전기적 위상각
odrv0.axis0.sensorless_estimator.phase_vel  # float32 | [rad/s] | 센서리스 추정 위상 각속도
odrv0.axis0.sensorless_estimator.pll_pos  # float32 | [rad] | 센서리스 PLL 추정 위치값
odrv0.axis0.sensorless_estimator.vel_estimate  # float32 | [turn/s] | 센서리스 추정 속도값
odrv0.axis0.step_dir_active  # bool | Step/Dir 입력 모드 활성 상태 여부
odrv0.axis0.steps  # int64 | Step/Dir 모드에서 누적된 현재 명령 위치(스텝 수)
odrv0.axis0.task_times.acim_estimator_update.end_time  # uint32 | ACIM 추정기 업데이트 태스크 종료 시각
odrv0.axis0.task_times.acim_estimator_update.length  # uint32 | ACIM 추정기 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.acim_estimator_update.start_time  # uint32 | ACIM 추정기 업데이트 태스크 시작 시각
odrv0.axis0.task_times.can_heartbeat.end_time  # uint32 | CAN 하트비트 태스크 종료 시각
odrv0.axis0.task_times.can_heartbeat.length  # uint32 | CAN 하트비트 태스크 실행 소요 시간
odrv0.axis0.task_times.can_heartbeat.start_time  # uint32 | CAN 하트비트 태스크 시작 시각
odrv0.axis0.task_times.controller_update.end_time  # uint32 | 컨트롤러 업데이트 태스크 종료 시각
odrv0.axis0.task_times.controller_update.length  # uint32 | 컨트롤러 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.controller_update.start_time  # uint32 | 컨트롤러 업데이트 태스크 시작 시각
odrv0.axis0.task_times.current_controller_update.end_time  # uint32 | 전류 제어 업데이트 태스크 종료 시각
odrv0.axis0.task_times.current_controller_update.length  # uint32 | 전류 제어 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.current_controller_update.start_time  # uint32 | 전류 제어 업데이트 태스크 시작 시각
odrv0.axis0.task_times.current_sense.end_time  # uint32 | 전류 센싱 태스크 종료 시각
odrv0.axis0.task_times.current_sense.length  # uint32 | 전류 센싱 태스크 실행 소요 시간
odrv0.axis0.task_times.current_sense.start_time  # uint32 | 전류 센싱 태스크 시작 시각
odrv0.axis0.task_times.dc_calib.end_time  # uint32 | DC 오프셋 캘리브레이션 태스크 종료 시각
odrv0.axis0.task_times.dc_calib.length  # uint32 | DC 오프셋 캘리브레이션 태스크 실행 소요 시간
odrv0.axis0.task_times.dc_calib.start_time  # uint32 | DC 오프셋 캘리브레이션 태스크 시작 시각
odrv0.axis0.task_times.encoder_update.end_time  # uint32 | 엔코더 업데이트 태스크 종료 시각
odrv0.axis0.task_times.encoder_update.length  # uint32 | 엔코더 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.encoder_update.start_time  # uint32 | 엔코더 업데이트 태스크 시작 시각
odrv0.axis0.task_times.endstop_update.end_time  # uint32 | 리미트 스위치 업데이트 태스크 종료 시각
odrv0.axis0.task_times.endstop_update.length  # uint32 | 리미트 스위치 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.endstop_update.start_time  # uint32 | 리미트 스위치 업데이트 태스크 시작 시각
odrv0.axis0.task_times.motor_update.end_time  # uint32 | 모터 업데이트 태스크 종료 시각
odrv0.axis0.task_times.motor_update.length  # uint32 | 모터 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.motor_update.start_time  # uint32 | 모터 업데이트 태스크 시작 시각
odrv0.axis0.task_times.open_loop_controller_update.end_time  # uint32 | 오픈루프 제어 업데이트 태스크 종료 시각
odrv0.axis0.task_times.open_loop_controller_update.length  # uint32 | 오픈루프 제어 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.open_loop_controller_update.start_time  # uint32 | 오픈루프 제어 업데이트 태스크 시작 시각
odrv0.axis0.task_times.pwm_update.end_time  # uint32 | PWM 업데이트 태스크 종료 시각
odrv0.axis0.task_times.pwm_update.length  # uint32 | PWM 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.pwm_update.start_time  # uint32 | PWM 업데이트 태스크 시작 시각
odrv0.axis0.task_times.sensorless_estimator_update.end_time  # uint32 | 센서리스 추정기 업데이트 태스크 종료 시각
odrv0.axis0.task_times.sensorless_estimator_update.length  # uint32 | 센서리스 추정기 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.sensorless_estimator_update.start_time  # uint32 | 센서리스 추정기 업데이트 태스크 시작 시각
odrv0.axis0.task_times.thermistor_update.end_time  # uint32 | 온도 센서 업데이트 태스크 종료 시각
odrv0.axis0.task_times.thermistor_update.length  # uint32 | 온도 센서 업데이트 태스크 실행 소요 시간
odrv0.axis0.task_times.thermistor_update.start_time  # uint32 | 온도 센서 업데이트 태스크 시작 시각
odrv0.brake_resistor_armed  # bool | 브레이크 저항 활성화(암) 상태 여부
odrv0.brake_resistor_current  # float32 | 브레이크 저항에 흐르는 전류값
odrv0.brake_resistor_saturated  # bool | 브레이크 저항 포화(한계 도달) 상태 여부
odrv0.can.error  # bitmask | CAN 버스 에러 코드
odrv0.error  # bitmask | 보드 전체 시스템 에러 코드
odrv0.fw_version_major  # uint8 | 펌웨어 버전 메이저 번호
odrv0.fw_version_minor  # uint8 | 펌웨어 버전 마이너 번호
odrv0.fw_version_revision  # uint8 | 펌웨어 버전 리비전 번호
odrv0.fw_version_unreleased  # uint8 | 미배포(개발) 펌웨어 여부
odrv0.hw_version_major  # uint8 | 하드웨어 버전 메이저 번호
odrv0.hw_version_minor  # uint8 | 하드웨어 버전 마이너 번호
odrv0.hw_version_variant  # uint8 | 하드웨어 버전 변형(variant) 번호
odrv0.ibus  # float32 | [A] | ODrive가 계산한 DC 버스 전류값
odrv0.misconfigured  # bool | 설정 오류(잘못된 구성) 감지 여부
odrv0.n_evt_control_loop  # uint32 | 제어 루프 실행 횟수
odrv0.n_evt_sampling  # uint32 | ADC 샘플링 실행 횟수
odrv0.oscilloscope.size  # uint32 | 오실로스코프 버퍼 크기
odrv0.otp_valid  # bool | OTP(공장 설정 메모리) 데이터 유효 여부
odrv0.serial_number  # uint64 | 보드 고유 시리얼 번호
odrv0.system_stats.i2c.addr  # uint8 | I2C 슬레이브 주소
odrv0.system_stats.i2c.addr_match_cnt  # uint32 | I2C 주소 일치(수신) 횟수
odrv0.system_stats.i2c.error_cnt  # uint32 | I2C 통신 에러 발생 횟수
odrv0.system_stats.i2c.rx_cnt  # uint32 | I2C 수신 바이트 수
odrv0.system_stats.max_stack_usage_analog  # uint32 | 아날로그 태스크 최대 스택 사용량
odrv0.system_stats.max_stack_usage_axis  # uint32 | 축 태스크 최대 스택 사용량
odrv0.system_stats.max_stack_usage_can  # uint32 | CAN 태스크 최대 스택 사용량
odrv0.system_stats.max_stack_usage_startup  # uint32 | 시작(startup) 태스크 최대 스택 사용량
odrv0.system_stats.max_stack_usage_uart  # uint32 | UART 태스크 최대 스택 사용량
odrv0.system_stats.max_stack_usage_usb  # uint32 | USB 태스크 최대 스택 사용량
odrv0.system_stats.min_heap_space  # uint32 | 최소 힙(동적 메모리) 여유 공간
odrv0.system_stats.prio_analog  # int32 | 아날로그 태스크 실행 우선순위
odrv0.system_stats.prio_axis  # int32 | 축 태스크 실행 우선순위
odrv0.system_stats.prio_can  # int32 | CAN 태스크 실행 우선순위
odrv0.system_stats.prio_startup  # int32 | 시작(startup) 태스크 실행 우선순위
odrv0.system_stats.prio_uart  # int32 | UART 태스크 실행 우선순위
odrv0.system_stats.prio_usb  # int32 | USB 태스크 실행 우선순위
odrv0.system_stats.stack_size_analog  # uint32 | 아날로그 태스크에 할당된 스택 크기
odrv0.system_stats.stack_size_axis  # uint32 | 축 태스크에 할당된 스택 크기
odrv0.system_stats.stack_size_can  # uint32 | CAN 태스크에 할당된 스택 크기
odrv0.system_stats.stack_size_startup  # uint32 | 시작(startup) 태스크에 할당된 스택 크기
odrv0.system_stats.stack_size_uart  # uint32 | UART 태스크에 할당된 스택 크기
odrv0.system_stats.stack_size_usb  # uint32 | USB 태스크에 할당된 스택 크기
odrv0.system_stats.uptime  # uint32 | 보드 부팅 후 경과 시간
odrv0.system_stats.usb.rx_cnt  # uint32 | USB 수신 바이트 수
odrv0.system_stats.usb.tx_cnt  # uint32 | USB 송신 바이트 수
odrv0.system_stats.usb.tx_overrun_cnt  # uint32 | USB 송신 버퍼 오버런 발생 횟수
odrv0.task_times.control_loop_checks.end_time  # uint32 | 제어 루프 점검 태스크 종료 시각
odrv0.task_times.control_loop_checks.length  # uint32 | 제어 루프 점검 태스크 실행 소요 시간
odrv0.task_times.control_loop_checks.start_time  # uint32 | 제어 루프 점검 태스크 시작 시각
odrv0.task_times.control_loop_misc.end_time  # uint32 | 제어 루프 기타 처리 태스크 종료 시각
odrv0.task_times.control_loop_misc.length  # uint32 | 제어 루프 기타 처리 태스크 실행 소요 시간
odrv0.task_times.control_loop_misc.start_time  # uint32 | 제어 루프 기타 처리 태스크 시작 시각
odrv0.task_times.dc_calib_wait.end_time  # uint32 | DC 캘리브레이션 대기 태스크 종료 시각
odrv0.task_times.dc_calib_wait.length  # uint32 | DC 캘리브레이션 대기 태스크 실행 소요 시간
odrv0.task_times.dc_calib_wait.start_time  # uint32 | DC 캘리브레이션 대기 태스크 시작 시각
odrv0.task_times.sampling.end_time  # uint32 | ADC 샘플링 태스크 종료 시각
odrv0.task_times.sampling.length  # uint32 | ADC 샘플링 태스크 실행 소요 시간
odrv0.task_times.sampling.start_time  # uint32 | ADC 샘플링 태스크 시작 시각
odrv0.user_config_loaded  # uint32 | 사용자 저장 설정이 정상적으로 로드되었는지 여부
odrv0.vbus_voltage  # float32 | [V] | ODrive가 측정한 DC 버스(입력 전원) 전압값
