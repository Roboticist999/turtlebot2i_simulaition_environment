# -*- coding: utf-8 -*-
"""
Created on Apr 16 18:11:07 2020

@author: Jack Hale
"""

"""
#####################
# RANSAC Algorithim #
#####################
"""

import numpy as np
import numpy.linalg as la
from parameters import parameters

def ransac(down_pcd, normals_pcd, static_radius=1,
           cylinder_radius=0, inlier=0, num_rounds=0,
           delta=0, local_zone=0):

    """
    This calls the parameters function and uses defaults to those values if
    none are specified when the ransac function is called
    """
    
    [voxel_size_temp, radius_temp, max_nn_temp, 
     cylinder_radius_temp, inlier_temp, num_rounds_temp, 
     delta_temp, local_zone_temp] = parameters()
    
    """
    This set of if statements allows us to specify different parameters when calling
    the ransac function. If no values are specified then the function defaults to 
    our parameters file
    """
    
    if cylinder_radius == 0:
        cylinder_radius = cylinder_radius_temp
    if inlier == 0:
        inlier = inlier_temp
    if num_rounds == 0:
        num_rounds = num_rounds_temp
    if delta == 0:
        delta = delta_temp
    if local_zone == 0:
        local_zone = local_zone_temp
    
    
    
        
    """
    This while loop is used to perform RANSAC on our point cloud
    Take a random sample of two points, where the second point is within a set distance 
        from the first random point
    Find the axis between the surface normals by taking the cross product
    Project all points onto a plane normal to the 'cylinder' axis
    Find all the points that fit about the axis difined from the first sampled point 
        to the axis (the cylinder radius) to a certain delta distance about that radius
    If the number of points found is above the inlier threshold we assume the object is a cylinder
    """
    
    # Determine the number of points in our down sampled point cloud
    if len(down_pcd) == 3:
        num_down_points = len(down_pcd.T)
    else:
        num_down_points = len(down_pcd)
    
    # Provide a transposed version of the point cloud
    down_switched = down_pcd.T
    
    i=0
    num_attempts = 0
    while i < inlier and num_attempts < num_rounds:
        """
        #############################
        # Finding Two Sample Points #
        #############################
        """
        
        C = 100 # Initialize C to test distance second sample is from the first
        sample_index = np.random.randint(len(down_pcd.T),size = 1) # Picks a random point in the point cloud
        # sample_index = np.array([616])
        sample_point = down_pcd[:,sample_index]
        sample_normal = normals_pcd[:,sample_index]
        
        """ 
        Determines the distance from the first to the second sample
        and continues until a sample close to the first is found.
        The distance away the second point can be is set by local_zone variable
        """
        while C > 0.15 and C != 0:
            second_index = np.random.randint(len(down_pcd.T),size=1) # np.array([4030])
            C = la.norm(down_pcd[:,second_index]-sample_point)
        # second_index = np.array([1390])
        # Set the second sample point and surface normal using the index found 
        second_sample = down_pcd[:,second_index]
        second_normal = normals_pcd[:,second_index]
        # sample_normal = sample_normal[np.newaxis]
        # second_normal = second_normal[np.newaxis]
        # Calulates a hypothetical cylinder axis from the two samples by taking the cross product
        sample_axis = np.cross(sample_normal.T,second_normal.T)
        
        
        """
        ########################
        # Finding Local Points #
        ########################
        """
        
        local_group = np.zeros([num_down_points,3])
        counter = 0
        for k in np.arange(num_down_points):
            distance = la.norm(down_pcd[:,k]-sample_point.T)
            if distance < local_zone:
                local_group[counter,:] = down_switched[k,:]
                counter += 1
        
        local_group = local_group[:counter,:]   
        
        """
        ######################
        # Cylinder Detection #
        ######################
        """
        
        # Initialize array to store the location of points that satisfy our cylinder constraints 
        cylinder_index = np.zeros(len(down_pcd.T))
        
        # Only runs if the number of points nearby is already greater than our specified
        # Inlier number
        if len(local_group) > inlier and la.norm(sample_axis)!=0:
            num_attempts+=1
            # Transpose our local group array so that it is 3xN
            local_group = local_group.T
        
            # Need to find unit vector of axis for our two sampled points
            norm_axis = sample_axis/la.norm(sample_axis)
            I = np.identity(3)
            # Create projection matrix. The @ sign does matrix multiplicaiton in numpy
            
            projection = I-norm_axis.T@norm_axis
            
            # Creates an array that is the location of all points projected onto a surface orthogonal to our cylinder
            x_plane = projection@local_group
            # Determines the location of the center point for our cylinder on the orthographic surface
            center = projection@sample_point+sample_normal*cylinder_radius
            # If our cylinder radius mode is dynamic than define the radius off the sampled point
            if static_radius == 0:
                # Define the radius of the cylinder to be from the axis to the first sampled point
                b = sample_point-second_sample
                a = np.concatenate((-sample_normal,second_normal), axis=1)
                x = la.lstsq(a,b, rcond=None)
                x = x[0]
                cylinder_radius = float(x[0])
                center = projection@sample_point+sample_normal*cylinder_radius
                # cylinder_radius = float(x)#la.norm(projection@sample_point-center.T)
            if cylinder_radius < 0.15:#local_zone*2:
                """
                Loops through all of the point cloud and determines 
                the points that satisfy our conditions. These conditions include:
                    Within the allowable noise of the cylinder radius
                    Within a distance from our first sampled point equal to the 
                    product of the cylinder radius and our length factor
                """
                
                indx = 0
                for k in np.arange(0,len(x_plane.T)):
                    proj_distance = la.norm(x_plane[:,k]-center.T) # Finds distance from point to center of projected circle
                    # distance = la.norm(local_group[:,k]-sample_point) # Finds distance from actual point to our first sample point
                    if (proj_distance < (cylinder_radius+delta)) and (proj_distance > (cylinder_radius-delta)):
                        # print(proj_distance)
                        cylinder_index[indx] = k # Stores viable point in an array for later use
                        indx+=1
                # The number of inliers found is equal to indx            
                i = indx
    
    
    
    # If there was no cylinder found return an empty 1D array
    if np.mean(cylinder_index) == 0:
        return [cylinder_index,0,0,0,0,0,0]
    
    cylinder_index = np.trim_zeros(cylinder_index) # Removes trailing zeros for cylinder_index
    viable_points = np.zeros([len(cylinder_index.T),3]) # Initialize an array to store the viable points found
    
    # Note, python arrays suck and we're going to transpose all of them so we can 
    # substitute columns. We need to do it as rows first aparently before we can 
    # make a substitutation
    
    # Transpose local group to insert it's respective points
    local_group_T = local_group.T
    
    # Determines the point in the original point cloud that corresponds to the 
    for j in np.arange(0, len(cylinder_index.T)):
        if cylinder_index[j] != 0:
            index = int(cylinder_index[j])
            viable_points[j,:] = local_group_T[index,:]
    viable_points = viable_points.T # Transpose viable points array to be 3xN
    
    return [viable_points, sample_point, second_sample, center, cylinder_radius, x_plane, norm_axis]
 
    