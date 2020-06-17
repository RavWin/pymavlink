unit uCopterData;

interface

Type

  TFlightMode=(fmRate, fmAngle, fmAltHolt, fmPosHold);
  TCopterMode=(cmBoot, cmSelfTest, cmSensorTest, cmLoadConfig, cmCalibration, cmInitIMU, cmReady);

  Point3D=packed record
    x,y,z: single;
  end;

  iPoint3D=packed record
    x,y,z: int16;
  end;

  GPSCoord=packed record
    lat:  single;
    lon:  single;
    alt:  single;
  end;

  TQuaternion=packed record
    case Integer of
    1: (q0: single;
    q1: single;
    q2: single;
    q3: single);
    2: (q : array[0..3] of Single);
  end;


  TConfigMotor=packed record
    matrixM: array[0..3, 0..3] of single;
    mixer:   array[0..3, 0..3] of single;
  end;

  TConfigIMU=packed record
    gXbias: Int16;
    gYbias: Int16;
    gZbias: Int16;
    aXbias: Int16;
    aYbias: Int16;
    aZbias: Int16;
    mXbias: Int16;
    mYbias: Int16;
    mZbias: Int16;
    gXscale: single;
    gYscale: single;
    gZscale: single;
    aXscale: single;
    aYscale: single;
    aZscale: single;
    mXscale: single;
    mYscale: single;
    mZscale: single;

    madgwickBeta: single;
  end;

  TConfigStab=packed record
    rateK: single;
    rateXp: single;
    rateYp: single;
    rateZp: single;
  end;

  //CCopter class structures representation

  TSensorData=packed record
    gyro: iPoint3D;
    acc: iPoint3D;
    mag: iPoint3D;
    gyroCalc: Point3D;
    accCalc: Point3D;
    magCalc: Point3D;
  end;
  PSensorData=^TSensorData;

  TDebugData=packed record
    data: array [0..7] of single;
  end;
  PDebugData=^TDebugData;

  TCopterState=packed record
    QAngle: TQuaternion;
    rate:   Point3D;
    attitude:  Point3D;
    heading:single;
    pos:    Point3D;
    vel:    Point3D;
    gpsPos: GPSCoord;
    gpsStart: GPSCoord;
    flightMode: TFlightMode;
    stateFlags: UInt32;
    copterMode: TCopterMode;
  end;
  PCopterState=^TCopterState;

  TCopterConfig=packed record
    IMU:  TConfigIMU;
    stab: TConfigStab;
    motorConfig: TConfigMotor;
  end;
  PCopterConfig=^TCopterConfig;

  TMotorData=packed record
    throttle: uint16;
    rpm:      uint16;
    temp:     uint16;
    state:    uint16;
  end;
  PMotorData=^TMotorData;
  TMotors= array [0..3] of TMotorData;
  PMotors=^TMotors;



implementation

end.