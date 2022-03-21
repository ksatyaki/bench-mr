import yaml
from matplotlib import image as mimage
from matplotlib import pyplot as plotty
import os


class OccMap:
    def __init__(self):
        self.image_file_name = ""
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.0
        self.image = None
        self.x_min = 0.0
        self.y_min = 0.0
        self.x_max = 0.0
        self.y_max = 0.0
        
    def load(self, map_yaml_filename):
        if not isinstance(map_yaml_filename, str):
            print("yamlToOccMapMsg needs a yaml file name as a string. Please provide the right parameters.")

        occmap_file_object = open(map_yaml_filename, 'r+')
        yaml_dict = yaml.load(occmap_file_object)

        self.image_file_name = yaml_dict['image']
        self.image = mimage.imread(map_yaml_filename[:map_yaml_filename.rfind(os.path.sep)] + os.path.sep + self.image_file_name)
        self.resolution = yaml_dict['resolution']
        self.origin_x = yaml_dict['origin'][0]
        self.origin_y = yaml_dict['origin'][1]

        self.x_min = self.origin_x
        self.y_min = self.origin_y
        self.x_max = self.origin_x + (
                self.image.shape[1] * self.resolution) + 1.0
        self.y_max = self.origin_y + (
                self.image.shape[0] * self.resolution) + 1.0

    def plot(self, axis=None, alpha=1.0):
        if axis is not None:
            axis.imshow(self.image, extent=[self.x_min, self.x_max, self.y_min, self.y_max], cmap='gray', alpha=alpha)
            axis.set_xlim([self.x_min, self.x_max])
            axis.set_ylim([self.y_min, self.y_max])
        else:
            plotty.imshow(self.image, extent=[self.x_min, self.x_max, self.y_min, self.y_max], cmap='gray', alpha=alpha)
            plotty.xlim([self.x_min, self.x_max])
            plotty.ylim([self.y_min, self.y_max])