#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2021

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
from PIL import Image
import numpy as np
import nrrd
from argparse import ArgumentParser


class NrrdConverter:
    def __init__(self, x_ratio, y_ratio, z_ratio):
        self._images_matrix = None
        self.num_segments = 0 # Number of segmentations, could be zero
        self.num_channels = 1 # Number of color channels, 1 for Grayscale and 4 for RGBA
        self.x_dim = 0
        self.y_dim = 0
        self.z_dim = 0
        self._segments_colors = None # Color of each segment that can be extracted from NRRD header
        self.nrrd_data = None
        self.nrrd_hdr = None
        self.x_dim_ratio = x_ratio
        self.y_dim_ratio = y_ratio
        self.z_dim_ratio = z_ratio

    def read_nrrd(self, filename):
        data, self.nrrd_hdr = nrrd.read(filename)
        z_step = self.z_dim_ratio
        y_step = self.y_dim_ratio
        x_step = self.x_dim_ratio

        self.nrrd_data = data[:, ::x_step, ::y_step, ::z_step]

    def initialize_image_matrix(self):
        # Create a 3D block where the first index refers to individual images,
        # The second index refers to x_dim, the third to y_dim and the fourth to z_dim(e.g. Z)

        self.num_segments = self.nrrd_data.shape[0]
        self.x_dim = self.nrrd_data.shape[1]
        self.y_dim = self.nrrd_data.shape[2]
        self.z_dim = self.nrrd_data.shape[3]

        if self.num_segments > 0:
            self.num_channels = 4
        else:
            self.num_channels = 1

        if self.num_channels == 1:
            self._images_matrix = np.zeros([self.x_dim, self.y_dim,  self.z_dim])
        if self.num_channels == 4:
            self._images_matrix = np.zeros([self.x_dim, self.y_dim, self.z_dim, self.num_channels])
            self._segments_colors = np.ones([self.num_segments, self.num_channels])
        else:
            # Throw some error or warning
            pass

    def create_segments_color_array_from_header(self):
        if self.num_channels > 0:
            for i in range(self.num_segments):
                query_str = 'Segment' + str(i) + '_Color'
                try:
                    color_str = self.nrrd_hdr[query_str]
                    self._segments_colors[i, 0:3] = np.fromstring(color_str, dtype=float, sep=' ')
                except KeyError:
                    raise KeyError
        else:
            # No need to create a color array for Grayscale
            pass

    def copy_volume_to_image_matrix(self):
        if self.num_channels == 4:
            for ns in range(self.num_segments):
                seg_data = self.nrrd_data[ns, :, :, :]
                print('Segment Number: ', ns)
                R = seg_data * self._segments_colors[ns, 0]
                print('\tStep: ', 1)
                G = seg_data * self._segments_colors[ns, 1]
                print ('\tStep: ', 2)
                B = seg_data * self._segments_colors[ns, 2]
                print('\tStep: ', 3)
                A = seg_data * self._segments_colors[ns, 3] * 0.7
                print('\tStep: ', 4)
                RGBA = np.stack((R, G, B, A), axis=-1)
                print('\tStep: ', 5)
                self._images_matrix += RGBA
                print('\tStep: ', 6)

    def normalize_image_matrix_data(self):
        max = self._images_matrix.max()
        min = self._images_matrix.min()
        self._images_matrix = (self._images_matrix - min) / float(max - min)

    def scale_image_matrix_data(self, scale):
        self._images_matrix = self._images_matrix * scale

    def normalize_data(self, data):
        max = data.max()
        min = data.min()
        normalized_data = (data - min) / float(max - min)
        return normalized_data

    def scale_data(self, data, scale):
        scaled_data = data * scale
        return scaled_data

    def save_image(self, array, im_name):
        im = Image.fromarray(array.astype(np.uint8))
        im.save(im_name)

    def save_image_matrix_as_images(self, im_prefix):
        for nz in range(self.z_dim):
            im_name = im_prefix + '0' + str(nz) + '.png'
            self.save_image(self._images_matrix[:, :, nz, :], im_name)


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-n', action='store', dest='nrrd_file', help='Specify Nrrd File')
    parser.add_argument('-p', action='store', dest='image_prefix', help='Specify Image Prefix', default='plane00')
    parser.add_argument('--rx', action='store', dest='x_skip', help='X axis order [1-100]. Higher value indicates greater reduction', default=1)
    parser.add_argument('--ry', action='store', dest='y_skip', help='Y axis order [1-100]. Higher value indicates greater reduction', default=1)
    parser.add_argument('--rz', action='store', dest='z_skip', help='Z axis order [1-100]. Higher value indicates greater reduction', default=1)
    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    nrrd_converter = NrrdConverter(int(parsed_args.x_skip), int(parsed_args.y_skip), int(parsed_args.z_skip))
    nrrd_converter.read_nrrd(parsed_args.nrrd_file)
    nrrd_converter.initialize_image_matrix()
    nrrd_converter.create_segments_color_array_from_header()
    nrrd_converter.copy_volume_to_image_matrix()
    # nrrd_converter.normalize_image_matrix_data()
    nrrd_converter.scale_image_matrix_data(255)
    nrrd_converter.save_image_matrix_as_images(parsed_args.image_prefix)


if __name__ == '__main__':
    main()
