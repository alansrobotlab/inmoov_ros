uint8    id             # motor id 0-11
float32  goal		# command position
float32  position	# current sensed position
float32  presentspeed	# calculated rotational speed
bool     moving		# is servo moving?
uint16   posraw		# raw position sensor value
bool     enabled	# is servo enabled?
bool     power		# does servo have power?


