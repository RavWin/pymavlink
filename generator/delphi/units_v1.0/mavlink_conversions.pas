unit mavlink_conversions;

interface
   uses uCopterData, mavlink_types, Math;
type
  TDCMmatrix = array[0..2,0..2] of Single;

implementation

procedure mavlink_quaternion_to_dcm(quaternion:TQuaternion ; var dcm:TDCMmatrix);
   var a,b,c,d,aSq,bSq,cSq,dSq : Double;
   begin
      a := quaternion.q0;
      b := quaternion.q1;
      c := quaternion.q2;
      d := quaternion.q3;
      aSq := a * a;
      bSq := b * b;
      cSq := c * c;
      dSq := d * d;
      dcm[0,0] := aSq + bSq - cSq - dSq;
      dcm[0,1] := 2 * (b * c - a * d);
      dcm[0,2] := 2 * (a * c + b * d);
      dcm[1,0] := 2 * (b * c + a * d);
      dcm[1,1] := aSq - bSq + cSq - dSq;
      dcm[1,2] := 2 * (c * d - a * b);
      dcm[2,0] := 2 * (b * d - a * c);
      dcm[2,1] := 2 * (a * b + c * d);
      dcm[2,2] := aSq - bSq - cSq + dSq;
   end;


procedure mavlink_dcm_to_euler(dcm:TDCMmatrix; var roll:Single; var pitch:Single; var yaw:Single);
    var phi, theta, psi : Single;
    begin
      theta := ArcSin(-dcm[2,0]);
      if (Abs(theta - Pi/2.0) < 1.0e-3) then
        begin
            phi := 0.0;
            psi := (ArcTan2(dcm[1,2] - dcm[0,1],
                    dcm[0,2] + dcm[1,1]) + phi);
        end
      else
        if (Abs(theta + Pi/2.0) < 1.0e-3) then
          begin
              phi := 0.0;
              psi := ArcTan2(dcm[1,2] - dcm[0,1],
                        dcm[0,2] + dcm[1,1] - phi);
          end
        else
          begin
            phi := ArcTan2(dcm[2,1], dcm[2,2]);
            psi := ArcTan2(dcm[1,0], dcm[0,0]);
          end;

      roll := phi;
      pitch := theta;
      yaw := psi;
    end;

procedure mavlink_quaternion_to_euler(quaternion : TQuaternion; var roll:Single; var pitch:Single; var yaw:Single);
    var dcm : TDCMmatrix;
    begin
        mavlink_quaternion_to_dcm(quaternion, dcm);
        mavlink_dcm_to_euler(dcm, roll, pitch, yaw);
    end;

procedure mavlink_euler_to_quaternion(roll:Single; pitch:Single; yaw:Single;var quaternion:TQuaternion);
    var cosPhi_2,sinPhi_2,cosTheta_2,sinTheta_2,cosPsi_2,sinPsi_2 :  Single;
    begin
       cosPhi_2 := Cos(roll / 2);
       sinPhi_2 := Sin(roll / 2);
       cosTheta_2 := Cos(pitch / 2);
       sinTheta_2 := Sin(pitch / 2);
       cosPsi_2 := Cos(yaw / 2);
       sinPsi_2 := Sin(yaw / 2);
       quaternion.q0 := (cosPhi_2 * cosTheta_2 * cosPsi_2 +
            sinPhi_2 * sinTheta_2 * sinPsi_2);
       quaternion.q1 := (sinPhi_2 * cosTheta_2 * cosPsi_2 -
            cosPhi_2 * sinTheta_2 * sinPsi_2);
       quaternion.q2 := (cosPhi_2 * sinTheta_2 * cosPsi_2 +
            sinPhi_2 * cosTheta_2 * sinPsi_2);
       quaternion.q3 := (cosPhi_2 * cosTheta_2 * sinPsi_2 -
            sinPhi_2 * sinTheta_2 * cosPsi_2);
    end;

procedure mavlink_dcm_to_quaternion(dcm:TDCMmatrix; var quaternion:TQuaternion);
  var s,tr : Single;
      dcm_i,i,dcm_j,dcm_k : Integer;
  begin
      tr := dcm[0,0] + dcm[1,1] + dcm[2,2];
      if (tr > 0.0) then begin
          s := Sqrt(tr + 1.0);
          quaternion.q0 := s * 0.5;
          s := 0.5 / s;
          quaternion.q1 := (dcm[2,1] - dcm[1,2]) * s;
          quaternion.q2 := (dcm[0,2] - dcm[2,0]) * s;
          quaternion.q3 := (dcm[1,0] - dcm[0,1]) * s;
      end
      else
      begin
          dcm_i := 0;
          for i := 1 to 2 do begin
              if (dcm[i,i] > dcm[dcm_i,dcm_i]) then
                  dcm_i := i;
          end;

          dcm_j := (dcm_i + 1) mod 3;
          dcm_k := (dcm_i + 2) mod 3;

          s := Sqrt((dcm[dcm_i,dcm_i] - dcm[dcm_j,dcm_j] -
                      dcm[dcm_k,dcm_k]) + 1.0);

          quaternion.q[dcm_i + 1] := s * 0.5;
          s := 0.5 / s;
          quaternion.q[dcm_j + 1] := (dcm[dcm_i,dcm_j] + dcm[dcm_j,dcm_i]) * s;
          quaternion.q[dcm_k + 1] := (dcm[dcm_k,dcm_i] + dcm[dcm_i,dcm_k]) * s;
          quaternion.q0 := (dcm[dcm_k,dcm_j] - dcm[dcm_j,dcm_k]) * s;
      end;
  end;

procedure mavlink_euler_to_dcm(roll:Single; pitch:Single; yaw:Single;var dcm:TDCMmatrix);
  var cosPhi,sinPhi,cosThe,sinThe,cosPsi,sinPsi : Single;
  Begin
      cosPhi := Cos(roll);
      sinPhi := Sin(roll);
      cosThe := Cos(pitch);
      sinThe := Sin(pitch);
      cosPsi := Cos(yaw);
      sinPsi := Sin(yaw);

      dcm[0,0] := cosThe * cosPsi;
      dcm[0,1] := -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
      dcm[0,2] := sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

      dcm[1,0] := cosThe * sinPsi;
      dcm[1,1] := cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
      dcm[1,2] := -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

      dcm[2,0] := -sinThe;
      dcm[2,1] := sinPhi * cosThe;
      dcm[2,2] := cosPhi * cosThe;
  end;

end.

