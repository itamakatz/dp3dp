; generated by Slic3r 1.2.9 on 2017-03-20 at 08:33:50

; external perimeters extrusion width = 0.50mm
; perimeters extrusion width = 0.50mm
; infill extrusion width = 0.50mm
; solid infill extrusion width = 0.50mm
; top infill extrusion width = 0.50mm

M107
M190 S80 ; set bed temperature
M104 S215 ; set temperature
G1 X3 Y0
G1 Z5 F5000 ; lift nozzle

M109 S215 ; wait for temperature to be reached
G21 ; set units to millimeters
G90 ; use absolute coordinates
M82 ; use absolute distances for extrusion
G92 E0
G1 Z0.500 F2400.000
G1 E-2.00000 F2400.00000
G92 E0
G1 X135.41094859722386 Y106.3515103321659 F2400.000
G1 E2.00000 F2400.00000
G1 X141.54901977760215 Y105.16012493615926 E2.69387 F1292.427
G1 X147.43176387739516 Y115.79263657575241 E3.38774
G1 X141.54901977760215 Y117.18455539191638 E4.08161
G1 X135.4639919868007 Y106.4462035228636 E4.76936
G1 X135.0578579720558 Y105.72063980924433 F2400.000
G1 X134.7038418160373 Y105.7961499648961 F2400.000
G1 X141.58080731511598 Y104.4772686809957 E5.54488 F1292.427
G1 X148.1388706585817 Y116.34799694302221 E6.32039
G1 X141.58080731511598 Y117.9173433942445 E7.09591
G1 X134.75688526008605 Y105.89084319839019 E7.86531
G1 X135.18685767854802 Y106.04610917548665 F2400.000
G1 X135.07012312869193 Y106.08382663514185 F2400.000
G1 X141.5639013378764 Y104.83066239508169 E8.59850 F1292.427
G1 X147.7725893459271 Y116.06032027277647 E9.33169
G1 X141.5639013378764 Y117.53739383320969 E10.06488
G1 X135.11609781591534 Y106.1786249347339 E10.79202
M104 S200 ; set temperature
G1 Z1.000 F2400.000
G1 X135.41094859722386 Y106.3515103321659 F2400.000
G1 X141.54901977760215 Y105.16012493615926 E11.48589 F900.000
G1 X147.43176387739516 Y115.79263657575241 E12.17976
G1 X141.54901977760215 Y117.18455539191638 E12.87363
G1 X135.4639919868007 Y106.4462035228636 E13.56138
G1 X135.0578579720558 Y105.72063980924433 F2400.000
G1 X134.7038418160373 Y105.7961499648961 F2400.000
G1 X141.58080731511598 Y104.4772686809957 E14.33690 F900.000
G1 X148.1388706585817 Y116.34799694302221 E15.11242
G1 X141.58080731511598 Y117.9173433942445 E15.88793
G1 X134.75688526008605 Y105.89084319839019 E16.65733
G1 X135.18685767854802 Y106.04610917548665 F2400.000
G1 X135.07012312869193 Y106.08382663514185 F2400.000
G1 X141.5639013378764 Y104.83066239508169 E17.39052 F1200.000
G1 X147.7725893459271 Y116.06032027277647 E18.12371
G1 X141.5639013378764 Y117.53739383320969 E18.85690
G1 X135.11609781591534 Y106.1786249347339 E19.58404
G1 E17.58404 F2400.00000
G92 E0
M104 S0 ; turn off temperature
G1 X3 Y0
M84     ; disable motors

; filament used = 17.6mm (0.0cm3)

; avoid_crossing_perimeters = 0
; bed_shape = 0x0,200x0,200x200,0x200
; bed_temperature = 80
; before_layer_gcode = 
; bridge_acceleration = 0
; bridge_fan_speed = 100
; brim_width = 0
; complete_objects = 0
; cooling = 1
; default_acceleration = 0
; disable_fan_first_layers = 3
; duplicate_distance = 6
; end_gcode = M104 S0 ; turn off temperature\nG28 X0  ; home X axis\nM84     ; disable motors\n
; extruder_clearance_height = 20
; extruder_clearance_radius = 20
; extruder_offset = 0x0
; extrusion_axis = E
; extrusion_multiplier = 1
; fan_always_on = 0
; fan_below_layer_time = 60
; filament_colour = #FF00FF
; filament_diameter = 1.75
; first_layer_acceleration = 0
; first_layer_bed_temperature = 80
; first_layer_extrusion_width = 0.5
; first_layer_speed = 30
; first_layer_temperature = 215
; gcode_arcs = 0
; gcode_comments = 0
; gcode_flavor = reprap
; infill_acceleration = 0
; infill_first = 0
; layer_gcode = 
; max_fan_speed = 100
; max_print_speed = 80
; max_volumetric_speed = 0
; min_fan_speed = 35
; min_print_speed = 10
; min_skirt_length = 0
; notes = 
; nozzle_diameter = 0.5
; only_retract_when_crossing_perimeters = 1
; ooze_prevention = 0
; output_filename_format = [input_filename_base].gcode
; perimeter_acceleration = 0
; post_process = 
; pressure_advance = 0
; resolution = 0
; retract_before_travel = 2
; retract_layer_change = 0
; retract_length = 2
; retract_length_toolchange = 10
; retract_lift = 0
; retract_restart_extra = 0
; retract_restart_extra_toolchange = 0
; retract_speed = 40
; skirt_distance = 6
; skirt_height = 1
; skirts = 0
; slowdown_below_layer_time = 5
; spiral_vase = 0
; standby_temperature_delta = -5
; start_gcode = G28 ; home all axes\nG1 Z5 F5000 ; lift nozzle\n
; temperature = 200
; threads = 2
; toolchange_gcode = 
; travel_speed = 40
; use_firmware_retraction = 0
; use_relative_e_distances = 0
; use_volumetric_e = 0
; vibration_limit = 0
; wipe = 0
; z_offset = 0
; dont_support_bridges = 1
; extrusion_width = 0.5
; first_layer_height = 0.5
; infill_only_where_needed = 0
; interface_shells = 0
; layer_height = 0.5
; raft_layers = 0
; seam_position = aligned
; support_material = 0
; support_material_angle = 0
; support_material_contact_distance = 0.2
; support_material_enforce_layers = 0
; support_material_extruder = 1
; support_material_extrusion_width = 0
; support_material_interface_extruder = 1
; support_material_interface_layers = 3
; support_material_interface_spacing = 0
; support_material_interface_speed = 100%
; support_material_pattern = pillars
; support_material_spacing = 2.5
; support_material_speed = 60
; support_material_threshold = 0
; xy_size_compensation = 0
; bottom_solid_layers = 3
; bridge_flow_ratio = 1
; bridge_speed = 60
; external_fill_pattern = rectilinear
; external_perimeter_extrusion_width = 0.5
; external_perimeter_speed = 50%
; external_perimeters_first = 0
; extra_perimeters = 1
; fill_angle = 45
; fill_density = 100%
; fill_pattern = rectilinear
; gap_fill_speed = 20
; infill_every_layers = 1
; infill_extruder = 1
; infill_extrusion_width = 0.5
; infill_overlap = 15%
; infill_speed = 40
; overhangs = 1
; perimeter_extruder = 1
; perimeter_extrusion_width = 0.5
; perimeter_speed = 40
; perimeters = 3
; small_perimeter_speed = 15
; solid_infill_below_area = 70
; solid_infill_every_layers = 0
; solid_infill_extruder = 1
; solid_infill_extrusion_width = 0.5
; solid_infill_speed = 20
; thin_walls = 1
; top_infill_extrusion_width = 0.5
; top_solid_infill_speed = 15
; top_solid_layers = 3