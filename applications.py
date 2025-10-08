import numpy as np
from scipy.signal import convolve2d


class PathFinder():

    def __init__(self, space_image, step_length=20):

        h_map = np.copy(space_image)
        h_map[h_map != 255] = -np.inf
        h_map[h_map == 255] = 0
        self.space = convolve2d(np.transpose(h_map), np.ones((10, 10)), mode='same')
        # self.space = np.transpose(h_map)
        self._step_length = step_length

    def find_route(self, origin, destination):

        origin = tuple(np.rint(origin).astype(int))
        destination = tuple(np.rint(destination).astype(int))
        
        fringe = list()
        parent_links = {origin: None}
        current_position = origin

        while destination not in parent_links:

            for neighbor in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
                expansion = tuple(np.add(current_position, neighbor))
                if all(0 <= expansion[_] < self.space.shape[_] for _ in range(2)):
                    if expansion not in parent_links and self.space[expansion] >= 0:
                        fringe.append(expansion)
                        parent_links[expansion] = current_position
        
            if fringe:
                current_position = fringe.pop(0)
            else:
                return None

        route = list()
        back_path = destination
        
        while back_path != origin:
            route.append(tuple(np.subtract(back_path, parent_links[back_path])))
            back_path = parent_links[back_path]

        route = np.pad(route[::-1], [(0, -len(route) % self._step_length), (0, 0)])
        route = route.reshape(-1, self._step_length, 2).sum(axis=1)

        return route.tolist()

    '''
    def set_temp_obs(self, y, x):

        if not np.isinf(self.space[y, x]):
            self.space[y, x] = -5

    def decay(self):

        self.space[self.space < 0] += 1
    '''
