#!/usr/bin/env python

import matplotlib.pyplot as plt

from map import Map
from editor import Editor

def draw(map2):
    lane_ids = []
    junction_ids = []
    map2.draw_lanes(plt, lane_ids)
    map2.draw_junctions(plt, junction_ids)
    map2.draw_crosswalks(plt)
    map2.draw_stop_signs(plt)
    map2.draw_yields(plt)

def show_map():
    map2 = Map()
    map2.load("data/borregas_ave.txt")
    draw(map2)
    plt.axis('equal')
    plt.show()

def save_map():
    pass

def load_base_map():
    pass

def add_editor():
    editor = Editor()
    fig.canvas.mpl_connect('button_press_event', editor.on_click)
    fig.canvas.mpl_connect('button_press_event', editor.on_press)
    fig.canvas.mpl_connect('button_release_event', editor.on_release)
    fig.canvas.mpl_connect('pick_event', editor.on_pick)
    fig.canvas.mpl_connect('motion_notify_event', editor.on_motion)

if __name__ == "__main__":
    fig, ax = plt.subplots()
    load_base_map()
    show_map()
    add_editor()
    save_map()
