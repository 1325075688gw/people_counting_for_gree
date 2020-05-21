
import sklearn.cluster as skc
class Cluster:
    def __init__(self):
        near_db = skc.DBSCAN(eps=0.2, min_samples=15)
        far_away_db = skc.DBSCAN(eps=0.2, min_samples=15)


    def get_frame_data(self):
        frame_data = common.queue_for_cluster_transfer.get()
        point_list = []
        for point in frame_data["point_list"]:
            # point_list.append(point)
            print(point)


        # """
        # 根据doppler过滤
        # :param frame_data: 每一帧的点云数据
        # :return: None
        # """
        # point_list = []
        # for point_dict in frame_data['point_list']:
        #     if point_dict['doppler'] != self.del_doppler:
        #         point_list.append(point_dict)
        # frame_data['point_list'] = point_list
        # frame_data['point_num'] = len(point_list)
        #
        # def frame_data_to_cluster_points(self, frame_data):
        #     points = []
        #     for data in frame_data['point_list']:
        #         point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
        #         points.append(point)
        #     return points