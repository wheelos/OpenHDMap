import matplotlib.pyplot as plt

from map import Map

def draw(map2):
    lane_ids = []
    junction_ids = []
    map2.draw_lanes(plt, lane_ids)
    map2.draw_junctions(plt, junction_ids)
    map2.draw_crosswalks(plt)
    map2.draw_stop_signs(plt)
    map2.draw_yields(plt)


if __name__ == "__main__":
    map2 = Map()
    map2.load("data/borregas_ave.txt")
    draw(map2)
    plt.axis('equal')
    plt.show()