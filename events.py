class Event():
  """Abstract class for traffic monitor events."""
  	def __init__(self, timestamp, actor):
		self.timestamp = timestamp
		self.actor = actor

class ArrivalEvent(Event):
  """Arrival of a vehicle at an intersection."""