string   joint          # joint name for status frame (populated in motor_status_dispatcher)
uint8    bus            # bus source for status frame (populated in motor_status_dispatcher)
uint8    id             # motor id 0-11
float32  goal           # command position
float32  position       # current sensed position
float32  presentspeed   # calculated rotational speed
bool     moving         # is servo moving?
uint16   posraw         # raw position sensor value
bool     enabled        # is servo enabled?
bool     power          # does servo have power?
uint8    temp           # current sensed temperature
uint8    error          # error flags
int16    value1         # custom value
int16    value2         # custom value
int16    value3         # custom value
int16    value4         # custom value
int16    value5         # custom value