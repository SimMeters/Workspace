------------------------------------------------------------------------------------------------------------------------
-- Copyright(c) 2016, SimMeters.com
-- All rights reserved. Released under the BSD license.
-- INERTHYA.lua 1.0 01/10/2015 (INERTHYA Protocol LUA Implementation)

-- Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
-- following conditions are met:

-- 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
-- following disclaimer.

-- 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
-- following disclaimer in the documentation and/or other materials provided with the distribution.

-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
-- INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
-- SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
-- SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
-- WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
-- USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

-- Data Output Protocol Format (DOPF) 10 Bytes Data
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+
--| BYTE9    | BYTE8    | BYTE7   | BYTE6     | BYTE5        | BYTE4        | BYTE3 | BYTE2 | BYTE1 | BYTE0 |
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+
--| CAN_IDH  | CAN_IDL  | NODE_ID | DATA_TYPE | SERVICE_CODE | MESSAGE_CODE | DATA3 | DATA2 | DATA1 | DATA0 |
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+

-- Data Input Protocol Format (DIPF) 10 Bytes Data
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+
--| BYTE9    | BYTE8    | BYTE7   | BYTE6     | BYTE5        | BYTE4        | BYTE3 | BYTE2 | BYTE1 | BYTE0 |
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+
--| CAN_IDH  | CAN_IDL  | NODE_ID | DATA_TYPE | SERVICE_CODE | MESSAGE_CODE | DATA3 | DATA2 | DATA1 | DATA0 |
--+----------+----------+---------+-----------+--------------+--------------+-------+-------+-------+-------+

-- CAN Aerospace V1.7 Constants and Functions
-- CAN Aerospace V1.7 NODES
------------------------------------------------------------------------------------------------------------------------
NODE_AHRS 		= 0x01
NODE_ADC 		= 0x02
NODE_VHF1 		= 0x03
NODE_VHF2 		= 0x04
NODE_NAVILS1    = 0x05
NODE_NAVILS2    = 0x06
NODE_ATC 		= 0x07
NODE_ADF 		= 0x08
NODE_GPS 		= 0x0A
NODE_DME 		= 0x0B
NODE_EMS 		= 0x0C
NODE_ETS 		= 0x0D
NODE_ES 		= 0x0E
NODE_WCS 		= 0x0F

------------------------------------------------------------------------------------------------------------------------
-- CAN Aerospace V1.7 DATA TYPES
------------------------------------------------------------------------------------------------------------------------
TYPE_NODATA		= 0x00
TYPE_ERROR 		= 0x01
TYPE_FLOAT 		= 0x02
TYPE_LONG 		= 0x03
TYPE_ULONG 		= 0x04
TYPE_BLONG 		= 0x05
TYPE_SHORT 		= 0x06
TYPE_USHORT     = 0x07
TYPE_BSHORT     = 0x08
TYPE_CHAR 		= 0x09
TYPE_UCHAR 		= 0x0A
TYPE_BCHAR 		= 0x0B
TYPE_SHORT2     = 0x0C
TYPE_USHORT2    = 0x0D
TYPE_BSHORT2    = 0x0E
TYPE_CHAR4 		= 0x0F
TYPE_UCHAR4     = 0x10
TYPE_BCHAR4     = 0x11
TYPE_CHAR2 		= 0x12
TYPE_UCHAR2     = 0x13
TYPE_BCHAR2     = 0x14
TYPE_MEMID 		= 0x15
TYPE_CHKSUM     = 0x16
TYPE_ACHAR 		= 0x17
TYPE_ACHAR2     = 0x18
TYPE_ACHAR4     = 0x19
TYPE_CHAR3 		= 0x1A
TYPE_UCHAR3     = 0x1B
TYPE_BCHAR3     = 0x1C
TYPE_ACHAR3     = 0x1D
TYPE_DOUBLEH    = 0x1E
TYPE_DOUBLEL    = 0x1F
TYPE_RESVD 		= 0x20
TYPE_UDEF 		= 0x64

------------------------------------------------------------------------------------------------------------------------
-- CAN Aerospace V1.7 SERVICE CODES
------------------------------------------------------------------------------------------------------------------------
SC0 			= 0x00
SC1 			= 0x01
SC2 			= 0x02
SC3 			= 0x03
SC4 			= 0x04
SC5 			= 0x05
SC6 			= 0x06
SC7 			= 0x07
SC8 			= 0x08

------------------------------------------------------------------------------------------------------------------------
-- CAN Aerospace V1.7 CAN ID's
------------------------------------------------------------------------------------------------------------------------
CA_BODY_LONGITUDINAL_ACCELERATION 						= 300
CA_BODY_LATERAL_ACCELERATION 							= 301
CA_BODY_NORMAL_ACCELERATION 							= 302
CA_BODY_PITCH_RATE										= 303
CA_BODY_ROLL_RATE										= 304
CA_BODY_YAW_RATE										= 305
CA_RUDDER_POSITION										= 306
CA_STABILIZER_POSITION									= 307
CA_ELEVATOR_POSITION									= 308
CA_LEFT_AILERON_POSITION								= 309
CA_RIGHT_AILERON_POSITION								= 310
CA_BODY_PITCH_ANGLE										= 311
CA_BODY_ROLL_ANGLE										= 312
CA_BODY_SIDESLIP										= 313
CA_ALTITUDE_RATE										= 314
CA_INDICATED_AIRSPEED									= 315
CA_TRUE_AIRSPEED										= 316
CA_CALIBRATED_AIRSPEED									= 317
CA_MACH_NUMBER											= 318
CA_BARO_CORRECTION										= 319
CA_BARO_CORRECTED_ALTITUDE								= 320
CA_HEADING_ANGLE										= 321
CA_STANDARD_ALTITUDE									= 322
CA_TOTAL_AIR_TEMPERATURE								= 323
CA_STATIC_AIR_TEMPERATURE								= 324
CA_DIFFERENTIAL_PRESSURE								= 325
CA_STATIC_PRESSURE										= 326
CA_HEADING_RATE											= 327
CA_PORT_SIDE_ANGLE_OF_ATTACK							= 328
CA_STARBORD_SIDE_ANGLE_OF_ATTACK						= 329
CA_DENSITY_ALTITUDE										= 330
CA_TURN_COORDINATION_RATE								= 331
CA_TRUE_ALTITUDE										= 332
CA_WIND_SPEED											= 333
CA_WIND_DIRECTION										= 334
CA_OUTSIDE_AIR_TEMPERATURE								= 335
CA_BODY_NORMAL_VELOCITY									= 336
CA_BODY_LONGITUDINAL_VELOCITY							= 337
CA_BODY_LATERAL_VELOCITY								= 338
CA_TOTAL_PRESSURE										= 339
CA_PITCH_CONTROL_POSITION								= 400
CA_ROLL_CONTROL_POSITION								= 401
CA_LATERAL_STICK_TRIM_POSITION_COMMAND					= 402
CA_YAW_CONTROL_POSITION									= 403
CA_COLLECTIVE_CONTROL_POSITION							= 404
CA_LONGITUDINAL_STICK_TRIM_POSITION_COMMAND				= 405
CA_DIRECTIONAL_PEDALS_TRIM_POSITION_COMMAND				= 406
CA_COLLECTIVE_LEVER_TRIM_POSITION_COMMAND 				= 407
CA_CYCLIC_CONTROL_STICK_SWITCHES 						= 408
CA_LATERAL_TRIM_SPEED 									= 409
CA_LONGITUDINAL_TRIM_SPEED 								= 410
CA_PEDAL_TRIM_SPEED 									= 411
CA_COLLECTIVE_TRIM_SPEED 								= 412
CA_NOSE_WHEEL_STEERING_HANDLE_POSITION 					= 413
CA_ENGINE_1_THROTTLE_LEVER_POSITION_ECS_CHANNEL_A		= 414
CA_ENGINE_2_THROTTLE_LEVER_POSITION_ECS_CHANNEL_A		= 415
CA_ENGINE_3_THROTTLE_LEVER_POSITION_ECS_CHANNEL_A		= 416
CA_ENGINE_4_THROTTLE_LEVER_POSITION_ECS_CHANNEL_A		= 417
CA_ENGINE_1_CONDITION_LEVER_POSITION_ECS_CHANNEL_A		= 418
CA_ENGINE_2_CONDITION_LEVER_POSITION_ECS_CHANNEL_A		= 419
CA_ENGINE_3_CONDITION_LEVER_POSITION_ECS_CHANNEL_A		= 420
CA_ENGINE_4_CONDITION_LEVER_POSITION_ECS_CHANNEL_A		= 421
CA_ENGINE_1_THROTTLE_LEVER_POSITION_ECS_CHANNEL_B		= 422
CA_ENGINE_2_THROTTLE_LEVER_POSITION_ECS_CHANNEL_B		= 423
CA_ENGINE_3_THROTTLE_LEVER_POSITION_ECS_CHANNEL_B		= 424
CA_ENGINE_4_THROTTLE_LEVER_POSITION_ECS_CHANNEL_B		= 425
CA_ENGINE_1_CONDITION_LEVER_POSITION_ECS_CHANNEL_B		= 426
CA_ENGINE_2_CONDITION_LEVER_POSITION_ECS_CHANNEL_B		= 427
CA_ENGINE_3_CONDITION_LEVER_POSITION_ECS_CHANNEL_B		= 428
CA_ENGINE_4_CONDITION_LEVER_POSITION_ECS_CHANNEL_B		= 429
CA_FLAPS_LEVER_POSITION 								= 430
CA_SLATS_LEVER_POSITION 								= 431
CA_PARK_BRAKE_LEVER_POSITION 							= 432
CA_SPEEDBRAKE_LEVER_POSITION 							= 433
CA_THROTTLE_MAX_LEVER_POSITION 							= 434
CA_PILOT_LEFT_BRAKE_PEDAL_POSITION 						= 435
CA_PILOT_RIGHT_BRAKE_PEDAL_POSITION 					= 436
CA_COPILOT_LEFT_BRAKE_PEDAL_POSITION 					= 437
CA_COPILOT_RIGHT_BRAKE_PEDAL_POSITION 					= 438
CA_TRIM_SYSTEM_SWITCHES 								= 439
CA_TRIM_SYSTEM_LIGHTS 									= 440
CA_COLLECTIVE_CONTROL_STICK_SWITCHES 					= 441
CA_STICK_SHAKER_STALL_WARNING_DEVICE 					= 442
CA_ENGINE_1_N1_ECS_CHANNEL_A 							= 500
CA_ENGINE_2_N1_ECS_CHANNEL_A 							= 501
CA_ENGINE_3_N1_ECS_CHANNEL_A 							= 502
CA_ENGINE_4_N1_ECS_CHANNEL_A 							= 503
CA_ENGINE_1_N2_ECS_CHANNEL_A 							= 504
CA_ENGINE_2_N2_ECS_CHANNEL_A 							= 505
CA_ENGINE_3_N2_ECS_CHANNEL_A 							= 506
CA_ENGINE_4_N2_ECS_CHANNEL_A 							= 507
CA_ENGINE_1_TORQUE_ECS_CHANNEL_A 						= 508
CA_ENGINE_2_TORQUE_ECS_CHANNEL_A 						= 509
CA_ENGINE_3_TORQUE_ECS_CHANNEL_A 						= 510
CA_ENGINE_4_TORQUE_ECS_CHANNEL_A 						= 511
CA_ENGINE_1_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_A		= 512
CA_ENGINE_2_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_A		= 513
CA_ENGINE_3_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_A		= 514
CA_ENGINE_4_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_A		= 515
CA_ENGINE_1_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_A		= 516
CA_ENGINE_2_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_A		= 517
CA_ENGINE_3_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_A		= 518
CA_ENGINE_4_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_A		= 519
CA_ENGINE_1_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A	= 520
CA_ENGINE_2_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A	= 521
CA_ENGINE_3_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A	= 522
CA_ENGINE_4_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A	= 523
CA_ENGINE_1_FUEL_FLOW_RATE_ECS_CHANNEL_A 				= 524
CA_ENGINE_2_FUEL_FLOW_RATE_ECS_CHANNEL_A 				= 525
CA_ENGINE_3_FUEL_FLOW_RATE_ECS_CHANNEL_A 				= 526
CA_ENGINE_4_FUEL_FLOW_RATE_ECS_CHANNEL_A 				= 527
CA_ENGINE_1_MANIFOLD_PRESSURE_ECS_CHANNEL_A				= 528
CA_ENGINE_2_MANIFOLD_PRESSURE_ECS_CHANNEL_A				= 529
CA_ENGINE_3_MANIFOLD_PRESSURE_ECS_CHANNEL_A				= 530
CA_ENGINE_4_MANIFOLD_PRESSURE_ECS_CHANNEL_A				= 531
CA_ENGINE_1_OIL_PRESSURE_ECS_CHANNEL_A 					= 532
CA_ENGINE_2_OIL_PRESSURE_ECS_CHANNEL_A 					= 533
CA_ENGINE_3_OIL_PRESSURE_ECS_CHANNEL_A 					= 534
CA_ENGINE_4_OIL_PRESSURE_ECS_CHANNEL_A 					= 535
CA_ENGINE_1_OIL_TEMPERATURE_ECS_CHANNEL_A 				= 536
CA_ENGINE_2_OIL_TEMPERATURE_ECS_CHANNEL_A 				= 537
CA_ENGINE_3_OIL_TEMPERATURE_ECS_CHANNEL_A 				= 538
CA_ENGINE_4_OIL_TEMPERATURE_ECS_CHANNEL_A 				= 539
CA_ENGINE_1_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_A		= 540
CA_ENGINE_2_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_A		= 541
CA_ENGINE_3_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_A		= 542
CA_ENGINE_4_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_A		= 543
CA_ENGINE_1_OIL_QUANTITY_ECS_CHANNEL_A 					= 544
CA_ENGINE_2_OIL_QUANTITY_ECS_CHANNEL_A 					= 545
CA_ENGINE_3_OIL_QUANTITY_ECS_CHANNEL_A 					= 546
CA_ENGINE_4_OIL_QUANTITY_ECS_CHANNEL_A 					= 547
CA_ENGINE_1_COOLANT_TEMPERATURE_ECS_CHANNEL_A 			= 548
CA_ENGINE_2_COOLANT_TEMPERATURE_ECS_CHANNEL_A 			= 549
CA_ENGINE_3_COOLANT_TEMPERATURE_ECS_CHANNEL_A 			= 550
CA_ENGINE_4_COOLANT_TEMPERATURE_ECS_CHANNEL_A 			= 551
CA_ENGINE_1_POWER_RATING_ECS_CHANNEL_A 					= 552
CA_ENGINE_2_POWER_RATING_ECS_CHANNEL_A 					= 553
CA_ENGINE_3_POWER_RATING_ECS_CHANNEL_A 					= 554
CA_ENGINE_4_POWER_RATING_ECS_CHANNEL_A 					= 555
CA_ENGINE_1_STATUS_1_ECS_CHANNEL_A 						= 556
CA_ENGINE_2_STATUS_1_ECS_CHANNEL_A 						= 557
CA_ENGINE_3_STATUS_1_ECS_CHANNEL_A 						= 558
CA_ENGINE_4_STATUS_1_ECS_CHANNEL_A 						= 559
CA_ENGINE_1_STATUS_2_ECS_CHANNEL_A 						= 560
CA_ENGINE_2_STATUS_2_ECS_CHANNEL_A 						= 561
CA_ENGINE_3_STATUS_2_ECS_CHANNEL_A 						= 562
CA_ENGINE_4_STATUS_2_ECS_CHANNEL_A 						= 563
CA_ENGINE_1_N1_ECS_CHANNEL_B 							= 564
CA_ENGINE_2_N1_ECS_CHANNEL_B 							= 565
CA_ENGINE_3_N1_ECS_CHANNEL_B 							= 566
CA_ENGINE_4_N1_ECS_CHANNEL_B 							= 567
CA_ENGINE_1_N2_ECS_CHANNEL_B 							= 568
CA_ENGINE_2_N2_ECS_CHANNEL_B 							= 569
CA_ENGINE_3_N2_ECS_CHANNEL_B 							= 570
CA_ENGINE_4_N2_ECS_CHANNEL_B 							= 571
CA_ENGINE_1_TORQUE_ECS_CHANNEL_B 						= 572
CA_ENGINE_2_TORQUE_ECS_CHANNEL_B 						= 573
CA_ENGINE_3_TORQUE_ECS_CHANNEL_B 						= 574
CA_ENGINE_4_TORQUE_ECS_CHANNEL_B 						= 575
CA_ENGINE_1_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_B		= 576
CA_ENGINE_2_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_B		= 577
CA_ENGINE_3_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_B		= 578
CA_ENGINE_4_TURBINE_INLET_TEMPERATURE_ECS_CHANNEL_B		= 579
CA_ENGINE_1_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_B		= 580
CA_ENGINE_2_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_B		= 581
CA_ENGINE_3_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_B		= 582
CA_ENGINE_4_INTER_TURBINE_TEMPERATURE_ECS_CHANNEL_B		= 583
CA_ENGINE_1_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B	= 584
ID_ENGINE_2_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B    = 585
ID_ENGINE_3_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B    = 586
ID_ENGINE_4_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B    = 587
CA_ENGINE_1_FUEL_FLOW_RATE_ECS_CHANNEL_B 				= 588
CA_ENGINE_2_FUEL_FLOW_RATE_ECS_CHANNEL_B 				= 589
CA_ENGINE_3_FUEL_FLOW_RATE_ECS_CHANNEL_B 				= 590
CA_ENGINE_4_FUEL_FLOW_RATE_ECS_CHANNEL_B 				= 591
CA_ENGINE_1_MANIFOLD_PRESSURE_ECS_CHANNEL_B 			= 592
CA_ENGINE_2_MANIFOLD_PRESSURE_ECS_CHANNEL_B 			= 593
CA_ENGINE_3_MANIFOLD_PRESSURE_ECS_CHANNEL_B 			= 594
CA_ENGINE_4_MANIFOLD_PRESSURE_ECS_CHANNEL_B 			= 595
CA_ENGINE_1_OIL_PRESSURE_ECS_CHANNEL_B 					= 596
CA_ENGINE_2_OIL_PRESSURE_ECS_CHANNEL_B 					= 597
CA_ENGINE_3_OIL_PRESSURE_ECS_CHANNEL_B 					= 598
CA_ENGINE_4_OIL_PRESSURE_ECS_CHANNEL_B 					= 599
CA_ENGINE_1_OIL_TEMPERATURE_ECS_CHANNEL_B 				= 600
CA_ENGINE_2_OIL_TEMPERATURE_ECS_CHANNEL_B 				= 601
CA_ENGINE_3_OIL_TEMPERATURE_ECS_CHANNEL_B 				= 602
CA_ENGINE_4_OIL_TEMPERATURE_ECS_CHANNEL_B 				= 603
CA_ENGINE_1_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_B		= 604
CA_ENGINE_2_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_B		= 605
CA_ENGINE_3_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_B		= 606
CA_ENGINE_4_CYLINDER_HEAD_TEMPERATURE_ECS_CHANNEL_B		= 607
CA_ENGINE_1_OIL_QUANTITY_ECS_CHANNEL_B 					= 608
CA_ENGINE_2_OIL_QUANTITY_ECS_CHANNEL_B 					= 609
CA_ENGINE_3_OIL_QUANTITY_ECS_CHANNEL_B 					= 610
CA_ENGINE_4_OIL_QUANTITY_ECS_CHANNEL_B 					= 611
CA_ENGINE_1_COOLANT_TEMPERATURE_ECS_CHANNEL_B 			= 612
CA_ENGINE_2_COOLANT_TEMPERATURE_ECS_CHANNEL_B 			= 613
CA_ENGINE_3_COOLANT_TEMPERATURE_ECS_CHANNEL_B 			= 614
CA_ENGINE_4_COOLANT_TEMPERATURE_ECS_CHANNEL_B 			= 615
CA_ENGINE_1_POWER_RATING_ECS_CHANNEL_B 					= 616
CA_ENGINE_2_POWER_RATING_ECS_CHANNEL_B 					= 617
CA_ENGINE_3_POWER_RATING_ECS_CHANNEL_B 					= 618
CA_ENGINE_4_POWER_RATING_ECS_CHANNEL_B 					= 619
CA_ENGINE_1_STATUS_1_ECS_CHANNEL_B 						= 620
CA_ENGINE_2_STATUS_1_ECS_CHANNEL_B 						= 621
CA_ENGINE_3_STATUS_1_ECS_CHANNEL_B 						= 622
CA_ENGINE_4_STATUS_1_ECS_CHANNEL_B 						= 623
CA_ENGINE_1_STATUS_2_ECS_CHANNEL_B 						= 624
CA_ENGINE_2_STATUS_2_ECS_CHANNEL_B 						= 625
CA_ENGINE_3_STATUS_2_ECS_CHANNEL_B 						= 626
CA_ENGINE_4_STATUS_2_ECS_CHANNEL_B 						= 627
CA_FUEL_PUMP_1_FLOW_RATE 								= 660
CA_FUEL_PUMP_2_FLOW_RATE 								= 661
CA_FUEL_PUMP_3_FLOW_RATE 								= 662
CA_FUEL_PUMP_4_FLOW_RATE 								= 663
CA_FUEL_PUMP_5_FLOW_RATE 								= 664
CA_FUEL_PUMP_6_FLOW_RATE 								= 665
CA_FUEL_PUMP_7_FLOW_RATE 								= 666
CA_FUEL_PUMP_8_FLOW_RATE 								= 667
CA_FUEL_TANK_1_QUANTITY 								= 668
CA_FUEL_TANK_2_QUANTITY 								= 669
CA_FUEL_TANK_3_QUANTITY 								= 670
CA_FUEL_TANK_4_QUANTITY 								= 671
CA_FUEL_TANK_5_QUANTITY 								= 672
CA_FUEL_TANK_6_QUANTITY 								= 673
CA_FUEL_TANK_7_QUANTITY 								= 674
CA_FUEL_TANK_8_QUANTITY 								= 675
CA_FUEL_TANK_1_TEMPERATURE 								= 676
CA_FUEL_TANK_2_TEMPERATURE 								= 677
CA_FUEL_TANK_3_TEMPERATURE 								= 678
CA_FUEL_TANK_4_TEMPERATURE 								= 679
CA_FUEL_TANK_5_TEMPERATURE 								= 680
CA_FUEL_TANK_6_TEMPERATURE 								= 681
CA_FUEL_TANK_7_TEMPERATURE 								= 682
CA_FUEL_TANK_8_TEMPERATURE 								= 683
CA_FUEL_SYSTEM_1_PRESSURE 								= 684
CA_FUEL_SYSTEM_2_PRESSURE 								= 685
CA_FUEL_SYSTEM_3_PRESSURE 								= 686
CA_FUEL_SYSTEM_4_PRESSURE 								= 687
CA_FUEL_SYSTEM_5_PRESSURE 								= 688
CA_FUEL_SYSTEM_6_PRESSURE 								= 689
CA_FUEL_SYSTEM_7_PRESSURE 								= 690
CA_FUEL_SYSTEM_8_PRESSURE 								= 691
CA_ROTOR_1_RPM 											= 700
CA_ROTOR_2_RPM 											= 701
CA_ROTOR_3_RPM 											= 702
CA_ROTOR_4_RPM 											= 703
CA_GEARBOX_1_SPEED 										= 704
CA_GEARBOX_2_SPEED 										= 705
CA_GEARBOX_3_SPEED 										= 706
CA_GEARBOX_4_SPEED 										= 707
CA_GEARBOX_5_SPEED 										= 708
CA_GEARBOX_6_SPEED 										= 709
CA_GEARBOX_7_SPEED 										= 710
CA_GEARBOX_8_SPEED 										= 711
CA_GEARBOX_1_OIL_PRESSURE 								= 712
CA_GEARBOX_2_OIL_PRESSURE 								= 713
CA_GEARBOX_3_OIL_PRESSURE 								= 714
CA_GEARBOX_4_OIL_PRESSURE 								= 715
CA_GEARBOX_5_OIL_PRESSURE 								= 716
CA_GEARBOX_6_OIL_PRESSURE 								= 717
CA_GEARBOX_7_OIL_PRESSURE 								= 718
CA_GEARBOX_8_OIL_PRESSURE 								= 719
CA_GEARBOX_1_OIL_TEMPERATURE 							= 720
CA_GEARBOX_2_OIL_TEMPERATURE 							= 721
CA_GEARBOX_3_OIL_TEMPERATURE 							= 722
CA_GEARBOX_4_OIL_TEMPERATURE 							= 723
CA_GEARBOX_5_OIL_TEMPERATURE 							= 724
CA_GEARBOX_6_OIL_TEMPERATURE 							= 725
CA_GEARBOX_7_OIL_TEMPERATURE 							= 726
CA_GEARBOX_8_OIL_TEMPERATURE 							= 727
CA_GEARBOX_1_OIL_QUANTITY 								= 728
CA_GEARBOX_2_OIL_QUANTITY 								= 729
CA_GEARBOX_3_OIL_QUANTITY 								= 730
CA_GEARBOX_4_OIL_QUANTITY 								= 731
CA_GEARBOX_5_OIL_QUANTITY 								= 732
CA_GEARBOX_6_OIL_QUANTITY 								= 733
CA_GEARBOX_7_OIL_QUANTITY 								= 734
CA_GEARBOX_8_OIL_QUANTITY 								= 735
CA_HYDRAULIC_SYSTEM_1_PRESSURE 							= 800
CA_HYDRAULIC_SYSTEM_2_PRESSURE 							= 801
CA_HYDRAULIC_SYSTEM_3_PRESSURE 							= 802
CA_HYDRAULIC_SYSTEM_4_PRESSURE 							= 803
CA_HYDRAULIC_SYSTEM_5_PRESSURE 							= 804
CA_HYDRAULIC_SYSTEM_6_PRESSURE 							= 805
CA_HYDRAULIC_SYSTEM_7_PRESSURE 							= 806
CA_HYDRAULIC_SYSTEM_8_PRESSURE 							= 807
CA_HYDRAULIC_SYSTEM_1_FLUID_TEMPERATURE 				= 808
CA_HYDRAULIC_SYSTEM_2_FLUID_TEMPERATURE 				= 809
CA_HYDRAULIC_SYSTEM_3_FLUID_TEMPERATURE 				= 810
CA_HYDRAULIC_SYSTEM_4_FLUID_TEMPERATURE 				= 811
CA_HYDRAULIC_SYSTEM_5_FLUID_TEMPERATURE 				= 812
CA_HYDRAULIC_SYSTEM_6_FLUID_TEMPERATURE 				= 813
CA_HYDRAULIC_SYSTEM_7_FLUID_TEMPERATURE 				= 814
CA_HYDRAULIC_SYSTEM_8_FLUID_TEMPERATURE 				= 815
CA_HYDRAULIC_SYSTEM_1_FLUID_QUANTITY 					= 816
CA_HYDRAULIC_SYSTEM_2_FLUID_QUANTITY 					= 817
CA_HYDRAULIC_SYSTEM_3_FLUID_QUANTITY 					= 818
CA_HYDRAULIC_SYSTEM_4_FLUID_QUANTITY 					= 819
CA_HYDRAULIC_SYSTEM_5_FLUID_QUANTITY 					= 820
CA_HYDRAULIC_SYSTEM_6_FLUID_QUANTITY 					= 821
CA_HYDRAULIC_SYSTEM_7_FLUID_QUANTITY 					= 822
CA_HYDRAULIC_SYSTEM_8_FLUID_QUANTITY 					= 823
CA_AC_SYSTEM_1_VOLTAGE 									= 900
CA_AC_SYSTEM_2_VOLTAGE 									= 901
CA_AC_SYSTEM_3_VOLTAGE 									= 902
CA_AC_SYSTEM_4_VOLTAGE 									= 903
CA_AC_SYSTEM_5_VOLTAGE 									= 904
CA_AC_SYSTEM_6_VOLTAGE 									= 905
CA_AC_SYSTEM_7_VOLTAGE 									= 906
CA_AC_SYSTEM_8_VOLTAGE 									= 907
CA_AC_SYSTEM_9_VOLTAGE 									= 908
CA_AC_SYSTEM_10_VOLTAGE 								= 909
CA_AC_SYSTEM_1_CURRENT 									= 910
CA_AC_SYSTEM_2_CURRENT 									= 911
CA_AC_SYSTEM_3_CURRENT 									= 912
CA_AC_SYSTEM_4_CURRENT 									= 913
CA_AC_SYSTEM_5_CURRENT 									= 914
CA_AC_SYSTEM_6_CURRENT 									= 915
CA_AC_SYSTEM_7_CURRENT 									= 916
CA_AC_SYSTEM_8_CURRENT 									= 917
CA_AC_SYSTEM_9_CURRENT 									= 918
CA_AC_SYSTEM_10_CURRENT 								= 919
CA_DC_SYSTEM_1_VOLTAGE 									= 920
CA_DC_SYSTEM_2_VOLTAGE 									= 921
CA_DC_SYSTEM_3_VOLTAGE 									= 922
CA_DC_SYSTEM_4_VOLTAGE 									= 923
CA_DC_SYSTEM_5_VOLTAGE 									= 924
CA_DC_SYSTEM_6_VOLTAGE 									= 925
CA_DC_SYSTEM_7_VOLTAGE 									= 926
CA_DC_SYSTEM_8_VOLTAGE 									= 927
CA_DC_SYSTEM_9_VOLTAGE 									= 928
CA_DC_SYSTEM_10_VOLTAGE 								= 929
CA_DC_SYSTEM_1_CURRENT 									= 930
CA_DC_SYSTEM_2_CURRENT 									= 931
CA_DC_SYSTEM_3_CURRENT 									= 932
CA_DC_SYSTEM_4_CURRENT 									= 933
CA_DC_SYSTEM_5_CURRENT 									= 934
CA_DC_SYSTEM_6_CURRENT 									= 935
CA_DC_SYSTEM_7_CURRENT 									= 936
CA_DC_SYSTEM_8_CURRENT 									= 937
CA_DC_SYSTEM_9_CURRENT 									= 938
CA_DC_SYSTEM_10_CURRENT 								= 939
CA_PROP_1_ICEGUARD_DC_CURRENT 							= 940
CA_PROP_2_ICEGUARD_DC_CURRENT 							= 941
CA_PROP_3_ICEGUARD_DC_CURRENT 							= 942
CA_PROP_4_ICEGUARD_DC_CURRENT 							= 943
CA_PROP_5_ICEGUARD_DC_CURRENT 							= 944
CA_PROP_6_ICEGUARD_DC_CURRENT 							= 945
CA_PROP_7_ICEGUARD_DC_CURRENT 							= 946
CA_PROP_8_ICEGUARD_DC_CURRENT 							= 947
CA_PROP_9_ICEGUARD_DC_CURRENT 							= 948
CA_PROP_10_ICEGUARD_DC_CURRENT 							= 949
CA_ACTIVE_NAV_SYSTEM_WAYPOINT_LATITUDE 					= 1000
CA_ACTIVE_NAV_SYSTEM_WAYPOINT_LONGITUDE 				= 1001
ID_ACTIVE_NAV_SYSTEM_WAYPOINT_HEIGHT_ABOVE_ELLIPSOID    = 1002
CA_ACTIVE_NAV_SYSTEM_WAYPOINT_ALTITUDE 					= 1003
CA_ACTIVE_NAV_SYSTEM_GROUND_SPEED 						= 1004
CA_ACTIVE_NAV_SYSTEM_TRUE_TRACK 						= 1005
CA_ACTIVE_NAV_SYSTEM_MAGNETIC_TRACK 					= 1006
CA_ACTIVE_NAV_SYSTEM_CROSS_TRACK_ERROR 					= 1007
CA_ACTIVE_NAV_SYSTEM_TRACK_ERROR_ANGLE 					= 1008
CA_ACTIVE_NAV_SYSTEM_TIME_TO_GO 						= 1009
CA_ACTIVE_NAV_SYSTEM_ESTIMATED_TIME_OF_ARRIVAL 			= 1010
CA_ACTIVE_NAV_SYSTEM_ESTIMATED_ENROUTE_TIME 			= 1011
CA_NAV_WAYPOINT_IDENTIFIER_0_3 							= 1012
CA_NAV_WAYPOINT_IDENTIFIER_4_7 							= 1013
CA_NAV_WAYPOINT_IDENTIFIER_8_11 						= 1014
CA_NAV_WAYPOINT_IDENTIFIER_12_15 						= 1015
CA_NAV_WAYPOINT_TYPE_IDENTIFIER 						= 1016
CA_NAV_WAYPOINT_LATITUDE 								= 1017
CA_NAV_WAYPOINT_LONGITUDE 								= 1018
CA_NAV_WAYPOINT_MINIMUM_ALTITUDE 						= 1019
CA_NAV_WAYPOINT_MINIMUM_FLIGHT_LEVEL 					= 1020
CA_NAV_WAYPOINT_MINIMUM_RADAR_HEIGHT 					= 1021
CA_NAV_WAYPOINT_MINIMUM_HEIGHT_ABOVE_ELLIPSOID 			= 1022
CA_NAV_WAYPOINT_MAXIMUM_ALTITUDE 						= 1023
CA_NAV_WAYPOINT_MAXIMUM_FLIGHT_LEVEL 					= 1024
CA_NAV_WAYPOINT_MAXIMUM_RADAR_HEIGHT 					= 1025
CA_NAV_WAYPOINT_MAXIMUM_HEIGHT_ABOVE_ELLIPSOID 			= 1026
CA_NAV_WAYPOINT_PLANNED_ALTITUDE 						= 1027
CA_NAV_WAYPOINT_PLANNED_FLIGHT_LEVEL 					= 1028
CA_NAV_WAYPOINT_PLANNED_RADAR_HEIGHT 					= 1029
CA_NAV_WAYPOINT_PLANNED_HEIGHT_ABOVE_ELLIPSOID 			= 1030
CA_DISTANCE_TO_NAV_WAYPOINT 							= 1031
CA_TIME_TO_GO_TO_NAV_WAYPOINT 							= 1032
CA_NAV_WAYPOINT_ESTIMATED_TIME_OF_ARRIVAL 				= 1033
CA_NAV_WAYPOINT_ESTIMATED_ENROUTE_TIME 					= 1034
CA_NAV_WAYPOINT_STATUS_INFORMATION 						= 1035
CA_GPS_AIRCRAFT_LATITUDE 								= 1036
CA_GPS_AIRCRAFT_LONGITUDE 								= 1037
CA_GPS_AIRCRAFT_HEIGHT_ABOVE_ELLIPSOID 					= 1038
CA_GPS_GROUND_SPEED 									= 1039
CA_GPS_TRUE_TRACK 										= 1040
CA_GPS_MAGNETIC_TRACK 									= 1041
CA_GPS_CROSS_TRACK_ERROR 								= 1042
CA_GPS_TRACK_ERROR_ANGLE 								= 1043
CA_GPS_GLIDESLOPE_DEVIATION 							= 1044
CA_GPS_PREDICTED_RAIM 									= 1045
CA_GPS_VERTICAL_FIGURE_OF_MERIT 						= 1046
CA_GPS_HORIZONTAL_FIGURE_OF_MERIT 						= 1047
CA_GPS_MODE_OF_OPERATION 								= 1048
CA_INS_AIRCRAFT_LATITUDE 								= 1049
CA_INS_AIRCRAFT_LONGITUDE 								= 1050
CA_INS_AIRCRAFT_HEIGHT_ABOVE_ELLIPSOID 					= 1051
CA_INS_AIRCRAFT_GROUND_SPEED 							= 1052
CA_INS_AIRCRAFT_TRUE_TRACK 								= 1053
CA_INS_AIRCRAFT_MAGNETIC_TRACK 							= 1054
CA_INS_AIRCRAFT_CROSS_TRACK_ERROR 						= 1055
CA_INS_AIRCRAFT_TRACK_ERROR_ANGLE 						= 1056
CA_INS_VERTICAL_FIGURE_OF_MERIT 						= 1057
CA_INS_HORIZONTAL_FIGURE_OF_MERIT 						= 1058
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_LATITUDE 				= 1059
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_LONGITUDE 				= 1060
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_HEIGHT_ABOVE_ELLIPSOID	= 1061
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_GROUND_SPEED			= 1062
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_TRUE_TRACK				= 1063
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_MAGNETIC_TRACK			= 1064
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_CROSS_TRACK_ERROR		= 1065
CA_AUXILIARY_NAV_SYSTEM_AIRCRAFT_TRACK_ERROR_ANGLE		= 1066
CA_AUXILIARY_NAV_SYSTEM_VERTICAL_FIGURE_OF_MERIT		= 1067
CA_AUXILIARY_NAV_SYSTEM_HORIZONTAL_FIGURE_OF_MERIT		= 1068
CA_MAGNETIC_HEADING 									= 1069
CA_RADIO_HEIGHT 										= 1070
CA_DME_1_DISTANCE 										= 1071
CA_DME_2_DISTANCE 										= 1072
CA_DME_3_DISTANCE 										= 1073
CA_DME_4_DISTANCE 										= 1074
CA_DME_1_TIME_TO_GO 									= 1075
CA_DME_2_TIME_TO_GO 									= 1076
CA_DME_3_TIME_TO_GO 									= 1077
CA_DME_4_TIME_TO_GO 									= 1078
CA_DME_1_GROUND_SPEED 									= 1079
CA_DME_2_GROUND_SPEED 									= 1080
CA_DME_3_GROUND_SPEED 									= 1081
CA_DME_4_GROUND_SPEED 									= 1082
CA_ADF_1_BEARING 										= 1083
CA_ADF_2_BEARING 										= 1084
CA_ADF_3_BEARING 										= 1085
CA_ADF_4_BEARING 										= 1086
CA_ILS_1_LOCALIZE_DEVIATION 							= 1087
CA_ILS_2_LOCALIZE_DEVIATION 							= 1088
CA_ILS_3_LOCALIZE_DEVIATION 							= 1089
CA_ILS_4_LOCALIZE_DEVIATION 							= 1090
CA_ILS_1_GLIDESLOPE_DEVIATION 							= 1091
CA_ILS_2_GLIDESLOPE_DEVIATION 							= 1092
CA_ILS_3_GLIDESLOPE_DEVIATION 							= 1093
CA_ILS_4_GLIDESLOPE_DEVIATION 							= 1094
CA_FLIGHT_DIRECTOR_1_PITCH_DEVIATION 					= 1095
CA_FLIGHT_DIRECTOR_2_PITCH_DEVIATION 					= 1096
CA_FLIGHT_DIRECTOR_1_ROLL_DEVIATION 					= 1097
CA_FLIGHT_DIRECTOR_2_ROLL_DEVIATION 					= 1098
CA_DECISION_HEIGHT 										= 1099
CA_VHF_1_COM_FREQUENCY 									= 1100
CA_VHF_2_COM_FREQUENCY 									= 1101
CA_VHF_3_COM_FREQUENCY 									= 1102
CA_VHF_4_COM_FREQUENCY 									= 1103
CA_VOR_ILS_1_FREQUENCY 									= 1104
CA_VOR_ILS_2_FREQUENCY 									= 1105
CA_VOR_ILS_3_FREQUENCY 									= 1106
CA_VOR_ILS_4_FREQUENCY 									= 1107
CA_ADF_1_FREQUENCY 										= 1108
CA_ADF_2_FREQUENCY 										= 1109
CA_ADF_3_FREQUENCY 										= 1110
CA_ADF_4_FREQUENCY 										= 1111
CA_DME_1_CHANNEL 										= 1112
CA_DME_2_CHANNEL 										= 1113
CA_DME_3_CHANNEL 										= 1114
CA_DME_4_CHANNEL 										= 1115
CA_TRANSPONDER_1_CODE 									= 1116
CA_TRANSPONDER_2_CODE 									= 1117
CA_TRANSPONDER_3_CODE 									= 1118
CA_TRANSPONDER_4_CODE 									= 1119
CA_DESIRED_TRACK_ANGLE 									= 1120
CA_MAGNETIC_VARIATION 									= 1121
CA_SELECTED_GLIDEPATH_ANGLE 							= 1122
CA_SELECTED_RUNWAY_HEADING 								= 1123
CA_COMPUTED_VERTICAL_VELOCITY 							= 1124
CA_SELECTED_COURSE 										= 1125
CA_VOR_1_RADIAL 										= 1126
CA_VOR_2_RADIAL 										= 1127
CA_VOR_3_RADIAL 										= 1128
CA_VOR_4_RADIAL 										= 1129
CA_TRUE_EAST_VELOCITY 									= 1130
CA_TRUE_NORTH_VELOCITY 									= 1131
CA_TRUE_UP_VELOCITY 									= 1132
CA_TRUE_HEADING 										= 1133
CA_GEAR_LEVER_SWITCHES 									= 1175
CA_GEAR_LEVER_LIGHTS_WOW_SOLENOID 						= 1176
CA_LANDING_GEAR_1_TIRE_PRESSURE 						= 1177
CA_LANDING_GEAR_2_TIRE_PRESSURE 						= 1178
CA_LANDING_GEAR_3_TIRE_PRESSURE 						= 1179
CA_LANDING_GEAR_4_TIRE_PRESSURE 						= 1180
CA_LANDING_GEAR_1_BRAKE_PAD_THICKNESS 					= 1181
CA_LANDING_GEAR_2_BRAKE_PAD_THICKNESS 					= 1182
CA_LANDING_GEAR_3_BRAKE_PAD_THICKNESS 					= 1183
CA_LANDING_GEAR_4_BRAKE_PAD_THICKNESS 					= 1184
CA_UTC 													= 1200
CA_CABIN_PRESSURE 										= 1201
CA_CABIN_ALTITUDE 										= 1202
CA_CABIN_TEMPERATURE 									= 1203
CA_LONGITUDINAL_CENTER_OF_GRAVITY 						= 1204
CA_LATERAL_CENTER_OF_GRAVITY 							= 1205
CA_DATE 												= 1206

------------------------------------------------------------------------------------------------------------------------
-- User Defined CAN ID's
------------------------------------------------------------------------------------------------------------------------
UD_RXDATA 												= 200
UD_FLAGS_0_31											= 210
UD_FLAGS_32_63 											= 211
UD_FLAGS_64_95 											= 212
UD_FLAGS_96_127                                      	= 213
UD_FLAGS_128_159 										= 214
UD_FLAGS_160_191 										= 215
UD_FLAGS_192_223 										= 216
UD_FLAGS_224_255 										= 217
-- ...
UD_OXYGEN_PRESSURE										= 1207
-- ...

------------------------------------------------------------------------------------------------------------------------
-- Variables
------------------------------------------------------------------------------------------------------------------------
MESSAGE_CODE_IDX = {}

------------------------------------------------------------------------------------------------------------------------
-- sbit pos
------------------------------------------------------------------------------------------------------------------------
function sbit(num, pos) 	
	return num + 2 ^ pos 
end

------------------------------------------------------------------------------------------------------------------------
-- Lshift (n) << by
------------------------------------------------------------------------------------------------------------------------
function lshift(x, by) 	
	return x * 2 ^ by 
end

------------------------------------------------------------------------------------------------------------------------
-- Rshift (n) >> by
------------------------------------------------------------------------------------------------------------------------
function rshift(x, by) 
	return math.floor(x / 2 ^ by) 
end

------------------------------------------------------------------------------------------------------------------------
-- ShortToBytes
------------------------------------------------------------------------------------------------------------------------
function ShortToBytes(v)
	return math.floor(v / 256), string.char(math.floor(v) % 256)
end

------------------------------------------------------------------------------------------------------------------------
-- genMessageCode
------------------------------------------------------------------------------------------------------------------------
function genMessageCode(can_id)
	if MESSAGE_CODE_IDX[can_id] ~= nil then
		MESSAGE_CODE_IDX[can_id] = math.floor(MESSAGE_CODE_IDX[can_id] + 1) % 256
	else
		MESSAGE_CODE_IDX[can_id] = 0
	end
	return MESSAGE_CODE_IDX[can_id]
end

------------------------------------------------------------------------------------------------------------------------
-- packFloat
------------------------------------------------------------------------------------------------------------------------
function packFloat(can_id, node_id, service_code, val)
	
	local sign = 0
    local v, byte = ""
    if val < 0 then sign = 1; val = -val end
    local mantissa, exponent = math.frexp(val)
    if val == 0 then mantissa = 0; exponent = 0 else
        mantissa = (mantissa * 2 - 1) * math.ldexp(0.5, 24)
        exponent = exponent + 126
    end
    val, byte = ShortToBytes(mantissa); v = v..byte
    val, byte = ShortToBytes(val); v = v..byte
    val, byte = ShortToBytes(exponent * 128 + val); v = v..byte
    val, byte = ShortToBytes(sign * 128 + val); v = v..byte
	return string.char(math.floor(can_id / 256), math.floor(can_id) % 256, node_id, TYPE_FLOAT, service_code, 
	genMessageCode(can_id), string.byte(v, 4), string.byte(v, 3), string.byte(v, 2), string.byte(v, 1))  									

end

------------------------------------------------------------------------------------------------------------------------
-- packUint32
------------------------------------------------------------------------------------------------------------------------
function packUint32(can_id, node_id, service_code, val)

	return string.char(math.floor(can_id / 256), math.floor(can_id) % 256, node_id, TYPE_ULONG, service_code, 
	genMessageCode(can_id), math.floor(rshift(val, 24)) % 256, math.floor(rshift(val, 16)) % 256, 
	math.floor(rshift(val, 8)) % 256, math.floor(val) % 256)  									

end