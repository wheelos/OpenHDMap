#!/usr/bin/env python

# draw_line
# draw_point
# edit_line
# pick_line
# pick_point

class Editor:
  def __init__(self):
    pass

  def on_click(self, event):
    pass

  def on_pick(self, event):
    if isinstance(event.artist, patches.Circle):
      if event.mouseevent.click:
        pass

  def on_press(self, event):
    pass

  def on_release(self, event):
    pass

  def on_motion(self, event):
    pass

