void serial_go_home()
{
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}
//#################################################################################################################################
void serial_clear_screen()
{
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}
//#################################################################################################################################
void serial_newline()
{
  Serial.write(27);
  Serial.print("\n");
}
//#################################################################################################################################

