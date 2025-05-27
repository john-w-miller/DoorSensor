# First pass at bias correction.  Works well when door is closed.  
When door is open, it tries to correct that as drift.  In hindsight, this may
work well. big positve change means the door is open. Big negative with steady 
accelerometer afterwards means door is closed.