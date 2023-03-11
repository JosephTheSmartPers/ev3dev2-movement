from datetime import datetime

now = datetime.now()

current_time = now.strftime("%S")
print("Current Time =", current_time)