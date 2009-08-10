class update_diagnostic_msgs_DiagnosticValue_250764bc9f6186841f2fd012ec0b7554(MessageUpdateRule):
	old_type = "diagnostic_msgs/DiagnosticValue"
	old_full_text = """
float32 value # a value to track over time
string label # what to label this value when viewing
"""

	new_type = "diagnostic_msgs/KeyValue"
	new_full_text = """
string value # a value to track over time
string label # what to label this value when viewing
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.value = str(old_msg.value)
		new_msg.label = old_msg.label

class update_diagnostic_msgs_DiagnosticString_160b8172585340ae9280191a0d54f07c(MessageUpdateRule):
	old_type = "diagnostic_msgs/DiagnosticString"
	old_full_text = """
string value # a string data type
string label # what to label this value when viewing
"""

	new_type = "diagnostic_msgs/KeyValue"
	new_full_text = """
string value # a value to track over time
string label # what to label this value when viewing
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.value = old_msg.value
		new_msg.label = old_msg.label

class update_diagnostic_msgs_DiagnosticMessage_a0876777999dce0cebd3c131da632df2(MessageUpdateRule):
	old_type = "diagnostic_msgs/DiagnosticMessage"
	old_full_text = """
Header header #for timestamp
DiagnosticStatus[] status # an array of components being reported on
================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
byte level #(OK=0, WARN=1, ERROR=2)
string name # a description of the test/component reporting
string message # a description of the status
DiagnosticValue[] values # an array of values associated with the status
DiagnosticString[] strings # an array of string associated with the status

================================================================================
MSG: diagnostic_msgs/DiagnosticValue
float32 value # a value to track over time
string label # what to label this value when viewing

================================================================================
MSG: diagnostic_msgs/DiagnosticString
string value # a string data type
string label # what to label this value when viewing
"""

	new_type = "diagnostic_msgs/DiagnosticArray"
	new_full_text = """
Header header #for timestamp
DiagnosticStatus[] status # an array of components being reported on
================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
byte level #(OK=0, WARN=1, ERROR=2)
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status


================================================================================
MSG: diagnostic_msgs/KeyValue
string value # a value to track over time
string label # what to label this value when viewing
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("DiagnosticStatus","DiagnosticStatus"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		self.migrate_array(old_msg.status, new_msg.status, "DiagnosticStatus")

class update_diagnostic_msgs_DiagnosticStatus_71cef332d5a23a49b5913222d03431df(MessageUpdateRule):
	old_type = "diagnostic_msgs/DiagnosticStatus"
	old_full_text = """
byte level #(OK=0, WARN=1, ERROR=2)
string name # a description of the test/component reporting
string message # a description of the status
DiagnosticValue[] values # an array of values associated with the status
DiagnosticString[] strings # an array of string associated with the status

================================================================================
MSG: diagnostic_msgs/DiagnosticValue
float32 value # a value to track over time
string label # what to label this value when viewing

================================================================================
MSG: diagnostic_msgs/DiagnosticString
string value # a string data type
string label # what to label this value when viewing
"""

	new_type = "diagnostic_msgs/DiagnosticStatus"
	new_full_text = """
byte level #(OK=0, WARN=1, ERROR=2)
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status


================================================================================
MSG: diagnostic_msgs/KeyValue
string value # a value to track over time
string label # what to label this value when viewing
"""

	order = 0
	migrated_types = [('DiagnosticValue','KeyValue'),
                          ('DiagnosticString','KeyValue')]

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.level = old_msg.level
		new_msg.name = old_msg.name
		new_msg.message = old_msg.message
		new_msg.hardware_id = 'NONE'
                tmp_values1 = []
                tmp_values2 = []
                self.migrate_array(old_msg.values, tmp_values1, 'KeyValue')
                self.migrate_array(old_msg.strings, tmp_values2, 'KeyValue')
		new_msg.values = tmp_values1 + tmp_values2


class update_diagnostic_msgs_KeyValue_160b8172585340ae9280191a0d54f07c(MessageUpdateRule):
	old_type = "diagnostic_msgs/KeyValue"
	old_full_text = """
string value # a value to track over time
string label # what to label this value when viewing
"""

	new_type = "diagnostic_msgs/KeyValue"
	new_full_text = """
string key # what to label this value when viewing
string value # a value to track over time
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.key = old_msg.label
		new_msg.value = old_msg.value

