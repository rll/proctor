import re
from pylab import *

class Bunch:
  def __init__(self, **kwds):
    self.__dict__.update(kwds)
  def dict(self):
    return self.__dict__

class Evaluation:
  def __init__(self, eval_set, feature):
    log_filename = eval_set.log_location_template%feature 
    self.feature = feature
    self.nice_name = feature.replace('_',' ')

    with open(log_filename) as log:
      lines = log.readlines()

    self.trials = []
    i = 0
    while i < len(lines):
      l = lines[i]

      # Model test output
      if l.find('[test') > -1:
        trial = Bunch() 
        trial.trial_num = int(re.search('test (\d+)', l).group(1))
        trial.model_ind = int(re.search('scanned model (\d+)', lines[i+1]).group(1))
        trial.model_at_rank = []
        try:
          for j in range(4):
            # get the first number on the line
            trial.model_at_rank.append(int(re.search('^(\d+)', lines[i+3+j]).group(1)))
          trial.detector_guess = int(re.search('detector guessed (\d+)', lines[i+3+4]).group(1))
          self.trials.append(trial)
        except:
          None
          # it's okay, detector just failed to find matches
        self.num_trials = len(self.trials)
      
      # Overview output
      if l.find('[overview]') > -1:
        m = re.search('^(\d+)', lines[i+1]) # first number
        self.num_correct = int(m.group(1))
        self.accuracy = 1.*self.num_correct / self.num_trials

      # Precision-Recall output
      if l.find('[precision-recall]') > -1:
        self.precision = []
        self.recall = []
        j = 1
        while True:
          if lines[i+j].find('[classifier stats]') > -1:
            break
          numbers = lines[i+j].split()
          self.precision.append(float(numbers[0]))
          self.recall.append(float(numbers[1]))
          j += 1
        self.ap = VOCap(self.recall,self.precision)

      # Some more statistics about the classifier
      if l.find('[classifier stats]') > -1:
        # get the last numbers in the next two lines
        self.avg_rank = float(lines[i+1].split()[-1])
        self.auh = float(lines[i+2].split()[-1])
        self.auh /= eval_set.num_models*self.num_trials

      # Confusion matrix
      if l.find('[confusion matrix]') > -1:
        # get the num_models lines following this one
        data = lines[i+1:i+1+eval_set.num_models]
        data = [line.strip() for line in data]
        self.confusion_matrix = matrix(';'.join(data))

      # Timing info
      if l.find('[timing]') > -1:
        print("TIMING HERE")
        # TODO: do timing info 
      i += 1

  def rank_histogram_data(self):
    # Go through and count the number of times the right model was rank-1,
    # rank-2, etc.
    # construct matrix of all the model_at_ranks
    all_model_at_rank = zeros((self.num_trials,4))
    correct_model = zeros((self.num_trials,1))
    for i,trial in enumerate(self.trials):
      all_model_at_rank[i,:] = trial.model_at_rank
      correct_model[i] = trial.model_ind
    # compare to the correct model
    correct_model_at_rank = all_model_at_rank == tile(correct_model,(1,4))
    # add a last column, which gets a 1 if none of the other columns have a 1
    catchall_rank_col = array(sum(correct_model_at_rank,1)>0)
    catchall_rank_col = transpose([catchall_rank_col]) # SERIOUSLY?
    correct_model_at_rank = concatenate((correct_model_at_rank,catchall_rank_col),1)
    counts = sum(correct_model_at_rank,0)
    return counts

def VOCap(rec,prec):
  mprec = hstack((0,prec,0))
  mrec = hstack((0,rec,1))
  # make sure prec is monotonically decreasing
  for i in range(len(mprec)-1,0,-1):
    mprec[i-1]=max(mprec[i-1],mprec[i])
  # find piecewise area under the curve
  i = add(nonzero(mrec[1:] != mrec[0:-1]),1)
  ap=sum((mrec[i]-mrec[subtract(i,1)])*mprec[i])
  return ap

