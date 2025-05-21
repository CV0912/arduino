double computeCost() {
  // Simulate the system for a short duration and compute the cost (SSE)
  double totalError = 0;
  for (int i = 0; i < 100; i++) {
    yawerror = yawtarget - yaw;  // Calculate error
    totalError += yawerror * yawerror;  // Accumulate squared error
    compute();  // Run the PID controller
    delay(10);  // Simulate sampling interval
  }
  return totalError / 100.0;  // Return average squared error
}

void tunePIDWithGradientDescent() {
  double currentCost = computeCost();
  double prevCost = currentCost + 10; // Initialize with a higher cost
  int step = 0;

  while (step < OPTIMIZATION_STEPS && abs(prevCost - currentCost) > CONVERGENCE_THRESHOLD) {
    prevCost = currentCost;

    // Test Kp
    double originalKp = kp;
    kp += LEARNING_RATE;  // Increment Kp
    double costWithKp = computeCost();
    double gradientKp = (costWithKp - currentCost) / LEARNING_RATE;
    kp = originalKp - LEARNING_RATE * gradientKp;  // Update Kp

    // Test Ki
    double originalKi = ki;
    ki += LEARNING_RATE;  // Increment Ki
    double costWithKi = computeCost();
    double gradientKi = (costWithKi - currentCost) / LEARNING_RATE;
    ki = originalKi - LEARNING_RATE * gradientKi;  // Update Ki

    // Test Kd
    double originalKd = kd;
    kd += LEARNING_RATE;  // Increment Kd
    double costWithKd = computeCost();
    double gradientKd = (costWithKd - currentCost) / LEARNING_RATE;
    kd = originalKd - LEARNING_RATE * gradientKd;  // Update Kd

    currentCost = computeCost(); // Recalculate current cost
    step++;

    // Debugging logs
    Serial.print("Step: ");
    Serial.print(step);
    Serial.print(", Cost: ");
    Serial.print(currentCost);
    Serial.print(", Kp: ");
    Serial.print(kp);
    Serial.print(", Ki: ");
    Serial.print(ki);
    Serial.print(", Kd: ");
    Serial.println(kd);
  }

  Serial.println("PID Tuning Complete");
}