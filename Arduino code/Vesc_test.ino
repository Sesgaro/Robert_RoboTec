void loop() {
    llanta1.spin();
    llanta2.spin();
    
    if (millis() - last_print_data > 1000) {
        Serial.print("rpm: ");
        Serial.println(can.erpm);
        Serial.print("voltage: ");
        Serial.println(can.inpVoltage);
        Serial.print("dutyCycleNow: ");
        Serial.println(can.dutyCycleNow);
        Serial.print("avgInputCurrent: ");
        Serial.println(can.avgInputCurrent);
        Serial.print("avgMotorCurrent: ");
        Serial.println(can.avgMotorCurrent);
        Serial.print("tempFET: ");
        Serial.println(can.tempFET);
        Serial.print("tempMotor: ");
        Serial.println(can.tempMotor);
        Serial.print("WattHours: ");
        Serial.println(can.WattHours);
        last_print_data = millis();
        Serial.println("..............................");
        Serial.println();
    }
    can.vesc_set_erpm(46);
    delay(5);
}
