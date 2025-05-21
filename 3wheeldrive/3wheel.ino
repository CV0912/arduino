
void compute3wheel() {
  // Calculate joystick-based velocities
  const double theta = -30 * pi / 180;
  double lx_rotated = lx * cos(theta) - ly * sin(theta);
  double ly_rotated = lx * sin(theta) + ly * cos(theta);
  double vx = ly_rotated;
  double vy = lx_rotated;
  w = rx;
 double speedFactorL = 2.625*(rL/44+rR/44)/2;    
    // Modify wheel velocities to include yaw correction
    target_wf = ((-0.5*vx) + (sqrt(3)/2)*vy + l0*w) * speedFactorL;
    target_wrr = ((-0.5*vx)-(sqrt(3)/2)*vy + l0*w) * speedFactorL;
    target_wrl = (vx+l0*w) * speedFactorL;
    // target_wf=(-0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
    // target_wrr = (0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
    // target_wrl = (-vx + l0*w)*speedFactorL;
    // Constrain target wheel speeds
    // target_wf = constrain(target_wf, -170, 170);
    // target_wrr = constrain(target_wrr, -170,170);
    // target_wrl = constrain(target_wrl, -170, 170);
}

void applyDeceleration() {
    // Gradually approach target speeds for each wheel
    current_wf += (target_wf-current_wf)/decelerationRate;
    current_wrl += (target_wrl-current_wrl)/decelerationRate;
    current_wrr += (target_wrr-current_wrr)/decelerationRate;
    
    // Serial.print("wf =");
    // Serial.print(current_wf);
    // Serial.print("wrr =");
    // Serial.print(current_wrr);
    // Serial.print("wrl =");
    // Serial.println(current_wrl);
    
    // Update motor directions a000000.nd speeds based on decelerated values
    if (current_wf >= 0) {
        digitalWrite(motorfrontdir, LOW);
        analogWrite(motorfrontpwm, current_wf);
    } else {
        digitalWrite(motorfrontdir, HIGH);
        analogWrite(motorfrontpwm, -current_wf);
    }
    
    if (current_wrl >= 0) {
        digitalWrite(motorreardirl, LOW);
        analogWrite(motorrearpwml, current_wrl);
    } else {
        digitalWrite(motorreardirl, HIGH);
        analogWrite(motorrearpwml, -current_wrl);
    }
    
    if (current_wrr >= 0) {
        digitalWrite(motorreardirr, LOW);
        analogWrite(motorrearpwmr, current_wrr);
    } else {
        digitalWrite(motorreardirr, HIGH);
        analogWrite(motorrearpwmr, -current_wrr);
    }
}