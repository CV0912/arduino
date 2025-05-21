void applyDeceleration() {
  // Gradually approach target speeds for each wheel
  current_wfl+= (target_wfl - current_wfl) / decelerationRate;
  current_wfr+= (target_wfr - current_wfr) / decelerationRate;
  current_wrl+= (target_wrl - current_wrl) / decelerationRate;
  current_wrr+= (target_wrr - current_wrr) / decelerationRate;
  
  // Update motor directions and speeds based on decelerated values
  if (current_wfl >= 0) {
    digitalWrite(motorfrontdir1, HIGH);
    analogWrite(motorfrontspeed1, current_wfl);
  } else {
    digitalWrite(motorfrontdir1, LOW);
    analogWrite(motorfrontspeed1, -current_wfl);
  }

  if (current_wfr >= 0) {
    digitalWrite(motorfrontdir2, HIGH);
    analogWrite(motorfrontspeed2, current_wfr);
  } else {
    digitalWrite(motorfrontdir2, LOW);
    analogWrite(motorfrontspeed2, -current_wfr);
  }

  if (current_wrl >= 0) {
    digitalWrite(motorbackdir1, HIGH);
    analogWrite(motorbackspeed1, current_wrl);
  } else {
    digitalWrite(motorbackdir1, LOW);
    analogWrite(motorbackspeed1, -current_wrl);
  }

  if (current_wrr >= 0) {
    digitalWrite(motorbackdir2, HIGH);
    analogWrite(motorbackspeed2, output);
  } else {
    digitalWrite(motorbackdir2, LOW);
    analogWrite(motorbackspeed2, -current_wrr);
  }
}
