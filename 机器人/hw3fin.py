# CSCI 3302: Homework 3 -- Clustering and Classification
# Implementations of K-Means clustering and K-Nearest Neighbor classification

# TODO: INSERT YOUR NAME HERE
LAST_NAME = "wang"

hw_data = "(dp0\nS'labels'\np1\n(lp2\nS'a'\np3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\nag3\naS'b'\np4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\nag4\naS'c'\np5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\nag5\naS'd'\np6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nag6\nasS'data'\np7\n(lp8\n(lp9\nF0.2281510519380156\naF1.8422288526327026\naa(lp10\nF-0.3120339505004928\naF-3.5401575578187696\naa(lp11\nF-1.6642876852162523\naF-1.166640077197513\naa(lp12\nF-0.5960371197386499\naF-1.5620862419869943\naa(lp13\nF0.8547750272557276\naF-1.5792905922807767\naa(lp14\nF-0.3471144136167954\naF-0.9812988120344468\naa(lp15\nF1.0314794589700145\naF-1.7208154778312896\naa(lp16\nF0.8232244593016377\naF-0.592884185712116\naa(lp17\nF0.47465778794096647\naF2.0109591022185547\naa(lp18\nF-0.09725630518508024\naF0.34251056671437996\naa(lp19\nF0.6287515327286145\naF3.41528320688084\naa(lp20\nF-1.8281441117929698\naF1.8187749118152894\naa(lp21\nF1.394366891615321\naF-1.191521470945223\naa(lp22\nF0.4001119983073187\naF-6.008412412791623\naa(lp23\nF-0.5861436615931658\naF2.9873299490290908\naa(lp24\nF0.23865713733052205\naF-0.6527531718351394\naa(lp25\nF1.3825618694527297\naF3.540333340241463\naa(lp26\nF-0.12086155747519574\naF-4.244027530927158\naa(lp27\nF-0.5736789470288806\naF2.510470571828508\naa(lp28\nF0.24334756044279324\naF-0.4695899035333493\naa(lp29\nF6.722194905669348\naF5.467131718501692\naa(lp30\nF4.5457681777175845\naF3.645631207375091\naa(lp31\nF3.782327642336134\naF2.850086187373674\naa(lp32\nF4.438672098330133\naF4.983852858670016\naa(lp33\nF3.9470020982494516\naF5.0124615671981445\naa(lp34\nF4.643670163892146\naF3.405710577824428\naa(lp35\nF5.170810705372265\naF4.001258267722553\naa(lp36\nF2.5811865700760146\naF4.127203182846279\naa(lp37\nF2.9157929678487733\naF3.941751840527834\naa(lp38\nF5.162027640907722\naF4.5829728939581615\naa(lp39\nF4.16846524901489\naF3.994855769541233\naa(lp40\nF3.5871416856320675\naF3.0383119276548487\naa(lp41\nF4.538987824379069\naF3.149106887686054\naa(lp42\nF3.4874505398761313\naF4.328878408586381\naa(lp43\nF3.4147181328008394\naF3.5705407654154726\naa(lp44\nF3.881691089458778\naF4.6694061577269315\naa(lp45\nF3.3618653205903293\naF4.567946698533381\naa(lp46\nF2.7725373008126026\naF4.9602783392764085\naa(lp47\nF4.400101595697098\naF3.954871493429828\naa(lp48\nF5.6427316619072565\naF3.4307815180310333\naa(lp49\nF-4.537608430990515\naF2.79160113735261\naa(lp50\nF-5.295060668067166\naF1.1297349429779548\naa(lp51\nF0.8414959000485389\naF3.503061658122245\naa(lp52\nF-3.135427436679456\naF3.3224057767732784\naa(lp53\nF-5.081733595759256\naF2.992060837479274\naa(lp54\nF-4.15255827693333\naF1.4266148484384102\naa(lp55\nF3.5484260900897002\naF2.8187847770585632\naa(lp56\nF-2.853068759182222\naF2.408739789901227\naa(lp57\nF-8.396601894945638\naF1.9797535058701072\naa(lp58\nF-1.5084831914039452\naF2.418934327528524\naa(lp59\nF-0.7936070236845021\naF1.4409147661295905\naa(lp60\nF-6.546937134670232\naF0.9709580146135508\naa(lp61\nF-3.93304759355755\naF3.356233283304774\naa(lp62\nF-1.5200518060547106\naF2.7123383763722035\naa(lp63\nF-8.458856989908172\naF0.32814173243937184\naa(lp64\nF-2.9995153269913537\naF2.7431079816945814\naa(lp65\nF-7.322064169453716\naF3.0243349198336027\naa(lp66\nF-3.8898723883731274\naF1.900824881531799\naa(lp67\nF-7.402487861699862\naF0.22015450259770875\naa(lp68\nF-6.883935065383595\naF1.845392014695902\naa(lp69\nF-0.4021568931053625\naF2.6375671142790056\naa(lp70\nF2.9472332038915283\naF-8.804122294837697\naa(lp71\nF1.0963975226698297\naF-1.0134144136327858\naa(lp72\nF-0.5624342980940387\naF-1.6546392977429896\naa(lp73\nF-0.5296146489109037\naF0.823146007645652\naa(lp74\nF1.0047200987063278\naF1.0248628858250646\naa(lp75\nF1.2494962049372769\naF1.0150000347480908\naa(lp76\nF1.5455472309633733\naF-3.2998517330497386\naa(lp77\nF1.9600687215617731\naF0.4957311329601879\naa(lp78\nF0.6767227056168389\naF-3.2925817180855983\naa(lp79\nF0.7732277453183324\naF-6.033047771015408\naa(lp80\nF-1.2660999928706116\naF1.5828447000203667\naa(lp81\nF2.2503082447698475\naF-3.4736202860277636\naa(lp82\nF-0.18422458764772753\naF-3.7212163990590743\naa(lp83\nF0.5068150354922949\naF-1.839585908495376\naa(lp84\nF2.940690860278959\naF-3.0435385944906255\naa(lp85\nF-0.08838342940889388\naF-0.6205215217694067\naa(lp86\nF1.6676315025593167\naF-0.8260205127555347\naa(lp87\nF1.974725280182308\naF-3.304180431305734\naa(lp88\nF2.0911614127439764\naF-1.7099145230031056\naas."








import pickle
import random
import copy
import pdb
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import cdist
import math

def visualize_data(data, cluster_centers_file):
  fig = plt.figure(1, figsize=(4,3))
  f = open(cluster_centers_file, 'rb')
  centers = pickle.load(f)
  f.close()

  km = KMeansClassifier()
  km._cluster_centers = centers

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

  labels = []
  center_colors = []
  for pt in data:
    labels.append(colors[km.classify(pt) % len(colors)])

  for i in range(len(centers)):
    center_colors.append(colors[i])

  plt.scatter([d[0] for d in data], [d[1] for d in data], c=labels, marker='x')
  plt.scatter([c[0] for c in centers], [c[1] for c in centers], c=center_colors, marker='D')
  plt.title("K-Means Visualization")
  plt.show()


class KMeansClassifier(object):

  def __init__(self):
    self._cluster_centers = [] # List of points representing cluster centers
    self._data = [] # List of datapoints (lists of real values)

  def add_datapoint(self, datapoint):
    self._data.append(datapoint)

  def fit(self, k):
    # Fit k clusters to the data, by starting with k randomly selected cluster centers.
    # HINT: To choose reasonable initial cluster centers, you can set them to be in the same spot as random (different) points from the dataset
   
    self._cluster_centers = random.sample(self._data, k)#randomly choose k cluster from self._data

    # TODO Follow convergence procedure to find final locations for each center

    # TODO Add each of the 'k' final cluster_centers to the model (self._cluster_centers)
    data_label = []
    for i in self._data:
      data_label.append(self.classify(i))#first convergence
    update = 1
    while update == 1:
      update = 0
      for i in range(len(self._data)):#compare the location belong to which cluster 
        if data_label[i] != self.classify(self._data[i]):#determine if that update
          data_label[i] = self.classify(self._data[i])
          update = 1
      self._cluster_centers = []
      for j in range(k):#0-k cluster
        tmp_center = []
        for i in range(len(self._data)):
          if data_label[i] == j:
            tmp_center.append(self._data[i])
        tmp_center = np.mean(tmp_center, 0)#calculate the mean
        self._cluster_centers.append([tmp_center[0], tmp_center[1]])#update the new center

 
  def classify(self,p):
    # Given a data point p, figure out which cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
    closest_cluster_index = 0
    # TODO Find nearest cluster center, then return its index in self._cluster_centers
    dist = cdist(self._cluster_centers, [p])
    closest_cluster_index = np.argmin(dist)#get the min distance

    return closest_cluster_index

class KNNClassifier(object):

  def __init__(self):
    self._data = [] # list of (data, label) tuples
  
  def clear_data(self):
    self._data = []

  def add_labeled_datapoint(self, data_point, label):
    self._data.append((data_point, label))
  
  def classify_datapoint(self, data_point, k):
    label_counts = {} # Dictionary mapping "label" => count
    best_label = None

    #TODO: Perform k_nearest_neighbor classification, set best_label to majority label for k-nearest points
    #5 steps: calculate the distance between cluster point and current distance
    labels = []
    for i in range(len(self._data)):
      labels.append(self._data[i][1])
    distance=[]
    for i in range(len(self._data)):
      x1 = data_point[0]
      y1 = data_point[1]
      x2 = self._data[i][0][0]
      y2 = self._data[i][0][1]
      distance.append(math.sqrt((x2 - x1)**2 + (y2 - y1)**2))
    #order the distance from small to high
    #select the min distance in k point
      order = np.argsort(distance)[:k]
    #first k point frequence
    for i in range(k):
      if labels[order[i]] in label_counts:
        label_counts[labels[order[i]]]+=1
      else:
        label_counts[labels[order[i]]]=1
    #return highest frequence cluster 
    best_label = max(label_counts, key=label_counts.get)

    return best_label



def print_and_save_cluster_centers(classifier, filename):
  for idx, center in enumerate(classifier._cluster_centers):
    print("  Cluster %d, center at: %s" % (idx, str(center)))


  f = open(filename,'wb')
  pickle.dump(classifier._cluster_centers, f)
  f.close()

def read_data_file(filename):
  f = open(filename)
  data_dict = pickle.load(f)
  f.close()

  return data_dict['data'], data_dict['labels']

def read_hw_data():
  global hw_data
  data_dict = pickle.loads(hw_data.encode())
  return data_dict['data'], data_dict['labels']

def main():
  global LAST_NAME
  # read data file
  #data, labels = read_data_file('hw3_data.pkl')

  # load dataset
  data, labels = read_hw_data()

  # data is an 'N' x 'M' matrix, where N=number of examples and M=number of dimensions per example
  # data[0] retrieves the 0th example, a list with 'M' elements
  # labels is an 'N'-element list, where labels[0] is the label for the datapoint at data[0]


  ########## PART 1 ############
  # perform K-means clustering
  kMeans_classifier = KMeansClassifier()
  for datapoint in data:
    kMeans_classifier.add_datapoint(datapoint) # add data to the model

  kMeans_classifier.fit(4) # Fit 4 clusters to the data

  # plot results
  print('\n'*2)
  print("K-means Classifier Test")
  print('-'*40)
  print("Cluster center locations:")
  print_and_save_cluster_centers(kMeans_classifier, "hw3_kmeans_" + LAST_NAME + ".pkl")

  print('\n'*2)


  ########## PART 2 ############
  print("K-Nearest Neighbor Classifier Test")
  print('-'*40)

  # Create and test K-nearest neighbor classifier
  kNN_classifier = KNNClassifier()
  k = 2

  correct_classifications = 0
  # Perform leave-one-out cross validation (LOOCV) to evaluate KNN performance
  for holdout_idx in range(len(data)):
    # Reset classifier
    kNN_classifier.clear_data()

    for idx in range(len(data)):
      if idx == holdout_idx: continue # Skip held-out data point being classified

      # Add (data point, label) tuples to KNNClassifier
      kNN_classifier.add_labeled_datapoint(data[idx], labels[idx])

    guess = kNN_classifier.classify_datapoint(data[holdout_idx], k) # Perform kNN classification
    if guess == labels[holdout_idx]: 
      correct_classifications += 1.0
  
  print("kNN classifier for k=%d" % k)
  print("Accuracy: %g" % (correct_classifications / len(data)))
  print('\n'*2)

  visualize_data(data, 'hw3_kmeans_' + LAST_NAME + '.pkl')


if __name__ == '__main__':
  main()
