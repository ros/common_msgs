class update_diagnostic_msgs_DiagnosticStatus_71cef332d5a23a49b5913222d03431df(MessageUpdateRule):
	old_type = "diagnostic_msgs/DiagnosticStatus"
	old_full_text = """
byte level #(OK=0, WARN=1, ERROR=2)
string name # a description of the test/component reporting
string message # a description of the status
KeyValue[] values # an array of values associated with the status
DiagnosticString[] strings # an array of string associated with the status

================================================================================
MSG: diagnostic_msgs/KeyValue
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
DiagnosticString[] strings # an array of string associated with the status

================================================================================
MSG: diagnostic_msgs/KeyValue
float32 value # a value to track over time
string label # what to label this value when viewing

================================================================================
MSG: diagnostic_msgs/DiagnosticString
string value # a string data type
string label # what to label this value when viewing
"""

	order = 0
	migrated_types = [
		("KeyValue","DiagnosticValue"),
		("DiagnosticString","DiagnosticString"),]

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.level = old_msg.level
		new_msg.name = old_msg.name
		new_msg.message = old_msg.message
		new_msg.hardware_id = 'NONE'
		self.migrate_array(old_msg.values, new_msg.values, "KeyValue")
		self.migrate_array(old_msg.strings, new_msg.strings, "DiagnosticString")

