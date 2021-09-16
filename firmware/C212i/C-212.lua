------------------------------------------------------------------------------------------------------------------------
-- Copyright(c) 2016, SimMeters.com
-- All rights reserved. Released under the BSD license.
-- C-212.lua 1.0 01/01/2021 (Import/Export script for Inerthya C-212 FTD)

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
------------------------------------------------------------------------------------------------------------------------
-- Includes and Variables
------------------------------------------------------------------------------------------------------------------------
dofile("INERTHYA.lua") 	-- SimMeters INERTHYA System LUA Implementation
etick = 0				-- Ticks for teh ET movement
etlat = 0				-- ET latch action

------------------------------------------------------------------------------------------------------------------------
-- Network Configuration 2 sockets 1 -> IESI / 1 -> MC Unit for TQS
------------------------------------------------------------------------------------------------------------------------
socket = require("socket")

TQS_CON = socket.try(socket.udp())
socket.try(TQS_CON:settimeout(.001))
socket.try(TQS_CON:setpeername("192.168.1.100", 6060))

GH3900_CON = socket.try(socket.udp())
socket.try(GH3900_CON:settimeout(.001))
socket.try(GH3900_CON:setpeername("127.0.0.1", 6060))

------------------------------------------------------------------------------------------------------------------------
-- Input Commands
------------------------------------------------------------------------------------------------------------------------
canOpsTable = {}

-- Set STD Pressure
canOpsTable[0x000001F6] = function() ipc.writeUW(0x0330, 16210) end

-- Set Barometric Pressure
canOpsTable[0x000001F8] = function() ipc.writeUW(0x0330, ipc.readUW(0x0330) + 1) end
canOpsTable[0x000001F9] = function() ipc.writeUW(0x0330, ipc.readUW(0x0330) - 1) end

-- Set Course
canOpsTable[0x000002F8] = function() ipc.writeUW(0x0C4E, ipc.readUW(0x0C4E) + 1) end
canOpsTable[0x000002F9] = function() ipc.writeUW(0x0C4E, ipc.readUW(0x0C4E) - 1) end

------------------------------------------------------------------------------------------------------------------------
-- Main Loop
------------------------------------------------------------------------------------------------------------------------
while true do

------------------------------------------------------------------------------------------------------------------------
-- EXPORT DATA FOR TQS
------------------------------------------------------------------------------------------------------------------------

	local TQS_OUT = ""

	-- Elevator trim control input: –16383 to +16383(Check Aircraft.cfg for tweak limits ??)
	TQS_OUT = TQS_OUT..packFloat(CA_ELEVATOR_POSITION, NODE_AHRS, 0, ipc.readSW(0x0BC0))

	-- Rudder trim value/control: –16383 to +16383 [NEW!](Check Aircraft.cfg for tweak limits ??)
	TQS_OUT = TQS_OUT..packFloat(CA_RUDDER_POSITION, NODE_AHRS, 0, ipc.readSW(0x0C04))

	-- Aileron trim value/control: –16383 to +16383 [NEW!](Check Aircraft.cfg for tweak limits ??)
	TQS_OUT = TQS_OUT..packFloat(CA_STABILIZER_POSITION, NODE_AHRS, 0, ipc.readSW(0x0C02))
	
	-- Elev Trim Wheel Control 07BC 4 Autopilot master switch and 07D0 4 Autopilot altitude lock
	if ((ipc.readSD(0x07BC) == 0) and (ipc.readSD(0x07D0) == 0)) then -- auto pilot off -> read pilot yoke value 0)TrimOFF / 1)TrimUP / 2)Trim Down
		TQS_OUT = TQS_OUT..packUint32(CA_TRIM_SYSTEM_SWITCHES, NODE_AHRS, 0, ipc.readUB(0x6002))
	else -- auto pilot on -> auto moving seq
		TQS_OUT = TQS_OUT..packUint32(CA_TRIM_SYSTEM_SWITCHES, NODE_AHRS, 0, 0)
	end

------------------------------------------------------------------------------------------------------------------------
-- TX Data TQS-MC
------------------------------------------------------------------------------------------------------------------------
	socket.try(TQS_CON:send(TQS_OUT))

------------------------------------------------------------------------------------------------------------------------
-- EXPORT / IMPORT DATA FOR GH3900
------------------------------------------------------------------------------------------------------------------------

	local GH3900_IN = ""
	local GH3900_OUT = ""

	-- Attitude indicator pitch value, in degrees. Double floating point format. 
	GH3900_OUT = GH3900_OUT..packFloat(CA_BODY_PITCH_ANGLE, NODE_AHRS, 0, ipc.readSD(0x0578) * (360 / (65536 * 65536)))

	-- Attitude indicator roll value, in degrees. Double floating point format.
	GH3900_OUT = GH3900_OUT..packFloat(CA_BODY_ROLL_ANGLE, NODE_AHRS, 0, ipc.readSD(0x057C) * (360 / (65536 * 65536)))

	-- Mach speed * 20480
	GH3900_OUT = GH3900_OUT..packFloat(CA_MACH_NUMBER, NODE_AHRS, 0, (ipc.readSW(0x11C6) / 20480))

	-- Turn co-ordinator ball position (slip and skid). –128 is extreme left, +127 is extreme right, 0 is balanced. (See 0374 for more accuracy)
	GH3900_OUT = GH3900_OUT..packFloat(CA_BODY_SIDESLIP, NODE_AHRS, 0, ipc.readSB(0x036E))

	-- IAS: Indicated Air Speed, as knots * 128
	GH3900_OUT = GH3900_OUT..packFloat(CA_INDICATED_AIRSPEED, NODE_AHRS, 0, (ipc.readSD(0x02BC) / 128))

	-- Flaps handle index (0 full up) (1 10%) (2 full down)
	GH3900_OUT = GH3900_OUT..packFloat(CA_FLAPS_LEVER_POSITION, NODE_AHRS, 0, ipc.readUB(0x0BFC))

	-- Altimeter pressure setting (“Kollsman” window). As millibars
	GH3900_OUT = GH3900_OUT..packFloat(CA_BARO_CORRECTION, NODE_AHRS, 0, (ipc.readUW(0x0330) / 16))

	-- This is the altimeter reading in feet (or metres, if the user is running with the preference for altitudes in metres), as a 32-bit signed integer.
	GH3900_OUT = GH3900_OUT..packFloat(CA_BARO_CORRECTED_ALTITUDE, NODE_AHRS, 0, ipc.readSD(0x3324))
	
	-- NAV1 OBS setting (degrees, 0–359)
	GH3900_OUT = GH3900_OUT..packFloat(CA_SELECTED_COURSE, NODE_AHRS, 0, ipc.readUW(0x0C4E))

	-- NAV1 Localiser Needle: –127 left to +127 right
	GH3900_OUT = GH3900_OUT..packFloat(CA_ILS_1_LOCALIZE_DEVIATION, NODE_AHRS, 0, ipc.readSB(0x0C48))

	-- NAV1 Glideslope Needle: –119 up to +119 down
	GH3900_OUT = GH3900_OUT..packFloat(CA_ILS_1_GLIDESLOPE_DEVIATION, NODE_AHRS, 0, ipc.readSB(0x0C49))

	-- GH3900 FLAGS
	lp = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

	if ipc.readDBL(0x2840) > 20 then 	-- Main bus voltage if great than 20V then ok
		lp[1] = 0
	else
		lp[1] = 1
	end

	lp[2] = ipc.readUB(0x0B67)			-- Fail mode: 0 ok, Attitude Indicator gauge inoperable = 1

	lp[3] = ipc.readUB(0x0B65)			-- Fail mode: 0 ok, ASI gauge inoperable = 1

	lp[4] = ipc.readUB(0x0B66)			-- Fail mode: 0 ok, Altimeter gauge inoperable = 1

	lp[5] = ipc.readUB(0x0D0C)			-- 0280 1 Lights: this operates the NAV, TAXI, PANEL and WING lights. For separate switches see offset 0D0C

	lp[6] = ipc.readUW(0x0BB0)			-- Outer Marker: activated when TRUE

	lp[7] = ipc.readUW(0x0BAE)			-- Middle Marker: activated when TRUE

	lp[8] = ipc.readUW(0x0BAC)			-- Inner Marker: activated when TRUE

	lp[9] = ipc.readUB(0x029C)			-- Pitot Heat switch (0=off, 1=on)

	GH3900_OUT = GH3900_OUT..packUint32(UD_FLAGS_0_31, NODE_AHRS, 0, (lp[32] * (2^31) + lp[31] * (2^30) + lp[30] * (2^29) + lp[29] * (2^28) + lp[28] * (2^27) + lp[27] * (2^26) + lp[26] * (2^25) + lp[25] * (2^24) + lp[24] * (2^23) + lp[23] * (2^22) + lp[22] * (2^21) + lp[21] * (2^20) + lp[20] * (2^19) + lp[19] * (2^18) + lp[18] * (2^17) + lp[17] * (2^16) + lp[16] * (2^15) + lp[15] * (2^14) + lp[14] * (2^13) + lp[13] * (2^12) + lp[12] * (2^11) + lp[11] * (2^10) + lp[10] * (2^9) + lp[9] * (2^8) + lp[8] * (2^7) + lp[7] * (2^6) + lp[6] * (2^5) + lp[5] * (2^4) + lp[4] * (2^3) + lp[3] * (2^2) + lp[2] * (2^1) + lp[1]))
	
	-- ipc.display("C-212.lua")
	-- ipc.display("GUST_LOCK 0x6200 -> "..ipc.readUB(0x6200))
	-- ipc.display("VS0:"..(ipc.readDBL(0x0538) * 0.5924838).."\nVS1:"..(ipc.readDBL(0x0540) * 0.5924838));

------------------------------------------------------------------------------------------------------------------------
-- TX Data GH3900
------------------------------------------------------------------------------------------------------------------------
	socket.try(GH3900_CON:send(GH3900_OUT))
	
------------------------------------------------------------------------------------------------------------------------
-- RX Data GH3900
------------------------------------------------------------------------------------------------------------------------
	GH3900_IN = GH3900_CON:receive()
	
	if GH3900_IN then

		CAN_IDH 		= string.byte(GH3900_IN,  1) -- CAN_IDH
		CAN_IDL 		= string.byte(GH3900_IN,  2) -- CAN_IDL
		NODE_ID 		= string.byte(GH3900_IN,  3) -- NODE ID RESERVED ALWAYS 0
		DATA_TYPE 		= string.byte(GH3900_IN,  4) -- DATA TYPE ALWAYS BCHAR
		SERVICE_CODE 	= string.byte(GH3900_IN,  5) -- SERVICE CODE DEVICE TYPE 0 UNKNOWN, 1 SWITCH, 2 ENCODER, 3 ANALOG
		MESSAGE_CODE 	= string.byte(GH3900_IN,  6) -- MESSAGE CODE COUNTER 0 - 255
		DATA3 			= string.byte(GH3900_IN,  7) -- ATA CHAPTER
		DATA2 			= string.byte(GH3900_IN,  8) -- ATA SUBCHAPTER
		DATA1 			= string.byte(GH3900_IN,  9) -- DEVICE NUMBER BUTTON0, BUTTON1, ENCODER2, ANALOG HSB
		DATA0 			= string.byte(GH3900_IN, 10) -- DEVICE VALUE ON/OFF, ENC LEFT/RIGHT , ANALOG LSB

		if SERVICE_CODE == SC1 or SERVICE_CODE == SC2 then 	-- SWITCH/ENCODER DEVICE
			
			if canOpsTable[((DATA3 * (16^6)) + (DATA2 * (16^4)) + (DATA1 * (16^2)) + DATA0)] ~= nil then
				
				canOpsTable[((DATA3 * (16^6)) + (DATA2 * (16^4)) + (DATA1 * (16^2)) + DATA0)]()
			
			end 
			
		elseif SERVICE_CODE == SC3 then 					-- ANALOG DEVICE

			if canOpsTable[(DATA3 * (16^2)) + DATA2] ~= nil then

				canOpsTable[(DATA3 * (16^2)) + DATA2]((DATA1 * (16^2)) + DATA0)

			end 

		end

	end

	ipc.sleep(5) -- delay
	
end