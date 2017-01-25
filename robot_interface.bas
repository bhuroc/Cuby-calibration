'!TITLE "Robot Interface"
PROGRAM RobotInterface

'#include <pacman.h>

TAKEARM
MOTOR ON
SPEED 20
EXTSPEED 50

ST_GRAVITY
ST_GRVOFFSET

ST_SETERALW 1, 20
ST_SETERALW 2, 20
ST_SETERALW 3, 30
ST_SETERALW 4, 70
ST_SETERALW 5, 20
ST_SETERALW 6, 30

ST_SETCURLMT 1, 30
ST_SETCURLMT 2, 50
ST_SETCURLMT 3, 60
ST_SETCURLMT 4, 100
ST_SETCURLMT 5, 100
ST_SETCURLMT 6, 100

I0=0
WHILE I0<>-99
	*getCommand:
		FLUSH #1
		INPUT #1,I0
		if I0=-98 then *goToPoint
		if I0=-97 then *changeToolOffset
		if I0=-96 then *getArea
		if I0=-95 then *goToPointAO
		if I0=-94 then *setWeight
		if I0=-93 then *setCurrentLimit
		if I0=-92 then *setOneAxisCurrentLimit
		if I0=-91 then *setDeflectionLimit
		if I0=-90 then *setInternalSpeed
		if I0=-89 then *gotoPointL
		if I0=-88 then *goToPointAOL
		if I0=-87 then *goPath
		if I0=-86 then *goTrjaectory
		if I0=-85 then *setExternalSpeed
		if I0=-84 then *goTrajectoryAO
wend

MOTOR OFF
END

*goPath:
INPUT #1,I1
FOR I2=1 TO I1
	INPUT #1, P[I2]
NEXT I2
MOVE P, P[1 TO I1]
PRINT #1,"Ready"
GOTO *getCommand

'goTrajectory also takes speed specifier
*goTrajectory:
INPUT #1,I1
FOR I2=1 TO I1
	INPUT #1, P[I2]
NEXT I2
INPUT #1,F1
SPEED F1
FOR I2=1 TO I1-1
	MOVE L,@P P[I2], NEXT
NEXT I2
MOVE P, P[I1]
PRINT #1, "Ready"
GOTO *getCommand

'goTrajectoryAO takes T type pose [3 vectors and 1 figure]
*goTrajectoryAO:
INPUT #1,I1
FOR I2=1 TO I1
	INPUT #1, T[I2]
NEXT I2
INPUT #1,F1
SPEED F1
FOR I2=1 TO I1-1
	MOVE L,@P T[I2], NEXT
NEXT I2
MOVE P, T[I1]
PRINT #1, "Ready"
GOTO *getCommand

*goToPointL:
	INPUT #1,F1
	LETX P99=F1
	INPUT #1,F1
	LETY P99=F1
	INPUT #1,F1
	LETZ P99=F1
	INPUT #1,F1
	LETRX P99=F1
	INPUT #1,F1
	LETRY P99=F1
	INPUT #1,F1
	LETRZ P99=F1
	INPUT #1,F1
	LETF P99=F1
	INPUT #1,F1
	F2 = POSX(P99) + POSY(P99) + POSZ(P99) + POSRX(P99) + POSRY(P99) + POSRZ(P99) + FIG(P99)
	F3 = ABS(F1-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		MOVE L,P99
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum", F2, F1
	END IF
GOTO *getCommand

*goToPoint:
	INPUT #1,F1
	LETX P99=F1
	INPUT #1,F1
	LETY P99=F1
	INPUT #1,F1
	LETZ P99=F1
	INPUT #1,F1
	LETRX P99=F1
	INPUT #1,F1
	LETRY P99=F1
	INPUT #1,F1
	LETRZ P99=F1
	INPUT #1,F1
	LETF P99=F1
	INPUT #1,F1
	F2 = POSX(P99) + POSY(P99) + POSZ(P99) + POSRX(P99) + POSRY(P99) + POSRZ(P99) + FIG(P99)
	F3 = ABS(F1-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		MOVE P,P99
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum", F2, F1
	END IF
	GOTO *getCommand


changeToolOffset:
	INPUT #1,F1
	LETX P98=F1
	INPUT #1,F1
	LETY P98=F1
	INPUT #1,F1
	LETZ P98=F1
	INPUT #1,F1
	LETRX P98=F1
	INPUT #1,F1
	LETRY P98=F1
	INPUT #1,F1
	LETRZ P98=F1
	INPUT #1,F1
	LETF P98=F1
	INPUT #1,F1
	F2 = POSX(P98) + POSY(P98) + POSZ(P98) + POSRX(P98) + POSRY(P98) + POSRZ(P98) + FIG(P98)
	F3 = ABS(F1-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		TOOL 1,P98
		CHANGETOOL 1
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum", F2, F1
	END IF
	GOTO *getCommand

*getArea:
	DIM center As Position
	DIM size As Vector
	center = AREAPOS(0)
	size = AREASIZE(0)
	PRINT #1,center," "
	PRINT #1,size," "
	PRINT #1,"Ready"
	GOTO *getCommand

*setExternalSpeed:
	INPUT #1,F1
	EXTSPEED F1
	PRINT #1, "Ready"
GOTO *getCommand

*setInternalSpeed:
	INPUT #1,F1
	SPEED F1
	PRINT #1, "Ready"
GOTO *getCommand

*goToPointAOL:
	DEFVEC tTA, tTO
	INPUT #1,F1
	LETX T49=F1
	INPUT #1,F1
	LETY T49=F1
	INPUT #1,F1
	LETZ T49=F1
	INPUT #1,F1
	LETX tTA=F1
	INPUT #1,F1
	LETY tTA=F1
	INPUT #1,F1
	LETZ tTA=F1
	LETA T49=tTA
	INPUT #1,F1
	LETX tTO=F1
	INPUT #1,F1
	LETY tTO=F1
	INPUT #1,F1
	LETZ tTO=F1
	LETO T49=tTO
	INPUT #1,F1
	LETF T49=F1
	INPUT #1,F1
	F2 = POSX(PVEC(T49)) + POSY(PVEC(T49)) + POSZ(PVEC(T49)) + POSX(tTA) + POSY(tTA) + POSZ(tTA) + POSX(tTO) + POSY(tTO) + POSZ(tTO) + FIG(T49)
	F3 = ABS(F1-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		MOVE L,T49
		PRINT #1,"Ready"
	ELSE
		PRINT #1, "Bad Checksum", F2, F1
	END IF
	GOTO *getCommand

*goToPointAO:
	DEFVEC lTA, lTO
	INPUT #1,F1
	LETX T49=F1
	INPUT #1,F1
	LETY T49=F1
	INPUT #1,F1
	LETZ T49=F1
	INPUT #1,F1
	LETX lTA=F1
	INPUT #1,F1
	LETY lTA=F1
	INPUT #1,F1
	LETZ lTA=F1
	INPUT #1,F1
	LETA T49=lTA
	LETX lTO=F1
	INPUT #1,F1
	LETY lTO=F1
	INPUT #1,F1
	LETZ lTO=F1
	LETO T49=lTO
	INPUT #1,F1
	LETF T49=F1
	INPUT #1,F1
	F2 = POSX(PVEC(T49)) + POSY(PVEC(T49)) + POSZ(PVEC(T49)) + POSX(lTA) + POSY(lTA) + POSZ(lTA) + POSX(lTO) + POSY(lTO) + POSZ(lTO) + FIG(T49)
	F3 = ABS(F1-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		MOVE P,T49
		PRINT #1,"Ready"
	ELSE
		PRINT #1, "Bad Checksum", F2, F1
	END IF
	GOTO *getCommand

*setWeight:
	INPUT #1,F10
	INPUT #1,F11
	INPUT #1,F12
	INPUT #1,F13
	INPUT #1,F6
	F2 = F10 + F11+F12+F13
	F3 = ABS(F6-F2)
	F4 = 0.0005
	IF F3 < F4 THEN
		ST_ASPACLD F10,F11,F12,F13
		ST_SETGRAVITY
		ST_SETGRVOFFSET
		PRINT #1,"Ready"
	ELSE
		Print #1, "Bad Checksum", F2, F6
	END IF
	GOTO *setCommand

*setCurrentLimit:
	INPUT #1,F1
	INPUT #1,F2
	F3 = ABS(F1-F2)
	IF F3<0.0005 THEN
		ST_SETCURLMT 1, F1
		ST_SETCURLMT 2, F1
		ST_SETCURLMT 3, F1
		ST_SETCURLMT 4, F1
		ST_SETCURLMT 5, F1
		ST_SETCURLMT 6, F1
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum",F1,F2
	END IF
goto *getCommand

*setOneAxisCurrentLimit:
	INPUT #1,F1
	INPUT #1,F2
	INPUT #1,F3
	F4 = F1 + F2
	F5 = ABS(F4-F3)
	IF F5<0.0005 THEN
		ST_SETCURLMT F1, F2
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum",F3, F4
	END IF
goto *getCommand

*setDeflectionLimit
	INPUT #1,F1
	INPUT #1,F2
	F3 = ABS(F1-F2)
	IF F3<0.0005 THEN
		ST_SETERALW 1, F1
		ST_SETERALW 2, F1
		ST_SETERALW 3, F1
		ST_SETERALW 4, F1
		ST_SETERALW 5, F1
		ST_SETERALW 6, F1
		PRINT #1, "Ready"
	ELSE
		PRINT #1, "Bad Checksum",F1,F2
	END IF
goto *getCommand
