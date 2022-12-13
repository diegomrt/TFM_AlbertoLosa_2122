### @brief Get the estimated positions of all pointcloud clusters
    ### @param num_samples - number of times to run the pipeline to get the cluster positions; these samples are then averaged together to get a more accurate result
    ### @param period - number of seconds to wait between sampling (give time for the backend to get updated with a new pointcloud)
    ### @param ref_frame - the desired reference frame the cluster positions should be transformed into; if unspecified, the camera's depth frame is used
    ### @param sort_axis - the axis of the 'ref_frame' by which to sort the cluster positions when returning them to the user
    ### @param reverse - if False, cluster positions are sorted in ascending order along the axis specified by 'sort_axis'; if True, the positions are sorted in descending order
    ### @param is_parallel - if False, the cluster positions returned to the user represent the centroids of each cluster w.r.t. the 'ref_frame';
    ###                      if True, the cluster positions returned to the user represent the centroids of each cluster, but positioned at the top of each cluster w.r.t. the 'ref_frame';
    ###                      set this to True if the 'ref_frame' is parallel to the surface that the objects are on
    ### @return <bool>, final_clusters - True if the algorithm succeeded or False otherwise. If False, the 'final_clusters' list is empty, but if True, a list of
    ###                                  dictionaries representing each cluster is returned to the user
    def get_cluster_positions(self, num_samples=5, period=0.1, ref_frame=None, sort_axis="y", reverse=False, is_parallel=True):
        root_clusters = self.srv_get_cluster_positions().clusters
        num_clusters = len(root_clusters)
        if num_clusters == 0:
            rospy.logwarn("No clusters found...")
            return False, []
        cluster_frame = root_clusters[0].frame_id

        # Calculate the average for each cluster based on 'num_samples' samples
        avg_clusters = [ClusterInfo() for i in range(num_clusters)]
        for x in range(num_samples):
            clusters = self.srv_get_cluster_positions().clusters
            if len(clusters) != num_clusters:
                rospy.logwarn("Found %d clusters instead of %d clusters..." % (len(clusters), num_clusters))
                return False, []
            valid_indices = list(range(num_clusters))
            for cluster in clusters:
                for indx in valid_indices:
                    if (abs(root_clusters[indx].position.x - cluster.position.x) < self.params.cluster_tol and
                       abs(root_clusters[indx].position.y - cluster.position.y) < self.params.cluster_tol and
                       abs(root_clusters[indx].position.z - cluster.position.z) < self.params.cluster_tol):
                       avg_clusters[indx].position.x += cluster.position.x / num_samples
                       avg_clusters[indx].position.y += cluster.position.y / num_samples
                       avg_clusters[indx].position.z += cluster.position.z /num_samples
                       avg_clusters[indx].color.r += cluster.color.r / num_samples
                       avg_clusters[indx].color.g += cluster.color.g / num_samples
                       avg_clusters[indx].color.b += cluster.color.b / num_samples
                       avg_clusters[indx].min_z_point.x += cluster.min_z_point.x / num_samples
                       avg_clusters[indx].min_z_point.y += cluster.min_z_point.y / num_samples
                       avg_clusters[indx].min_z_point.z += cluster.min_z_point.z / num_samples
                       avg_clusters[indx].num_points += cluster.num_points / float(num_samples)
                       valid_indices.remove(indx)
                       break
                    elif (indx == num_clusters - 1):
                        rospy.logwarn("Could not match the cluster. Please tune the filter parameters such that all spherical 'object markers' are constant in their respective clusters and do not flicker.")
                        return False, []
            rospy.sleep(period)

        # Get the transform from the 'ref_frame' to the cluster frame (i.e. the camera's depth frame) - known as T_rc
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform(ref_frame, cluster_frame, rospy.Time(0), rospy.Duration(4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (ref_frame, cluster_frame))
            return False, []
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        q = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(q)
        T_rc = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        # Sort the clusters from left to right w.r.t. the camera's depth frame
        sorted_cam_clusters = sorted(avg_clusters, key=lambda cluster : cluster.position.x)

        # TO DO...
        # if is_parallel is True...
        # find 'yaw' of each cluster relative to a virtual frame located at 'cluster_frame' (with the 'x' axis
        # pointed in the same direction, but with the 'z' axis pointing straight up) using OpenCV (order is left to right)
        # set this 'yaw' to the yaw variable in each cluster; it will be added to the yaw part of T_rc before the transform is published

        # Transform the clusters to be w.r.t. the 'ref_frame' instead of the camera's depth frame
        for cluster in sorted_cam_clusters:
            # p_co is the cluster's position w.r.t. the camera's depth frame
            # p_ro is the cluster's position w.r.t. the desired reference frame
            p_co = [cluster.position.x, cluster.position.y, cluster.position.z, 1]
            p_ro = np.dot(T_rc, p_co)
            if (is_parallel):
                # p_comin is the minimum point of the cluster (in the 'z' direction) w.r.t. the camera's depth frame;
                # it is assumed that this point lies at the top or very near the top of the cluster
                # p_romin is the same point w.r.t. the desired reference frame; the 'z' element of this point replaces
                # the 'z' element in p_ro; thus, a tf frame published at this point should appear at the 'top-center' of the cluster
                p_comin = [cluster.min_z_point.x, cluster.min_z_point.y, cluster.min_z_point.z, 1]
                p_romin = np.dot(T_rc, p_comin)
                p_ro[2] = p_romin[2]
            cluster.position.x = p_ro[0]
            cluster.position.y = p_ro[1]
            cluster.position.z = p_ro[2]

        # Sort the clusters based on user input
        if sort_axis == "x":
            key = lambda cluster : cluster.position.x
        elif sort_axis == "y":
            key = lambda cluster : cluster.position.y
        elif sort_axis == "z":
            key = lambda cluster : cluster.position.z
        else:
            key = lambda cluster : cluster.position.y
            rospy.logwarn("'%s' is not a valid sorting axis. Set the 'sort_axis' argument to 'x', 'y', or 'z'. Defaulting to 'y'." % sort_axis)
        sorted_ref_clusters = sorted(sorted_cam_clusters, key=key, reverse=reverse)

        # publish transforms to the /tf tree for debugging purposes (only once)
        final_trans = []
        cluster_num = 1
        time_now = rospy.Time.now()
        for cluster in sorted_ref_clusters:
            trans = TransformStamped()
            trans.header.frame_id = ref_frame
            trans.header.stamp = time_now
            trans.child_frame_id = "cluster_" + str(cluster_num)
            trans.transform.translation.x = cluster.position.x
            trans.transform.translation.y = cluster.position.y
            trans.transform.translation.z = cluster.position.z
            trans.transform.rotation = Quaternion(0, 0, 0, 1)
            final_trans.append(trans)
            cluster_num += 1
        self.br.sendTransform(final_trans)

        # create a list of Python dictionaries to return to the user
        final_clusters = []
        for indx in range(num_clusters):
            name = final_trans[indx].child_frame_id
            x = final_trans[indx].transform.translation.x
            y = final_trans[indx].transform.translation.y
            z = final_trans[indx].transform.translation.z
            yaw = 0
            r = sorted_ref_clusters[indx].color.r
            g = sorted_ref_clusters[indx].color.g
            b = sorted_ref_clusters[indx].color.b
            num_points = sorted_ref_clusters[indx].num_points
            cluster = {"name" : name, "position" : [x, y, z], "yaw" : yaw, "color" : [r, g, b], "num_points" : num_points}
            final_clusters.append(cluster)
        return True, final_clusters