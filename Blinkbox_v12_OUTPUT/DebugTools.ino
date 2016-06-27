#ifdef DEBUG_TOOLS

ErrorMonitor::ErrorMonitor( void )
{
  _destroyed = true;
}

void ErrorMonitor::begin(uint8_t graphWidth, float maxPossibleError, float minPossibleError, float centerOn)
{
  _graphWidth = round(graphWidth);
  _maxPossibleError = round(maxPossibleError);
  _minPossibleError = round(minPossibleError);
  _centerOn = round(centerOn);
  _errorBuffer[graphWidth];
  _index = 0;
    
  _destroyed = false;
}

void ErrorMonitor::end()
{
  _destroyed = true;
}

bool ErrorMonitor::appendError( float error )
{
  if(_index >= _graphWidth)
  {
    return false;
  }
  _errorBuffer[_index++] = round(error);
  return true;
}

void ErrorMonitor::plotTable()
{
  Serial.print("MAX : ");
  Serial.println( _maxPossibleError );
  Serial.print("MIN : ");
  Serial.println( _minPossibleError );
  Serial.print("CEN : ");
  Serial.println( _centerOn );
  
  Serial.print("[");
  for(int i=0; i<_graphWidth; i++)
  {
    Serial.print(_errorBuffer[i]);
    Serial.print(",");
  }
  Serial.println("]");
}

#endif
