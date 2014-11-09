/**
 * Reads the input from the IRSensor port. This number is from 0-1023, so we convert this number into a float from 0.0 to 5.0 volts.
 */
float getIRVoltage()
{
  return analogRead(IRSensor) * (5.0 / 1023.0);
}
