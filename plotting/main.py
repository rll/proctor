#!/usr/bin/env python
"""
Processes all data output by Proctor to output all plots and Latex-source
tables that we could need for publication.
"""
import re
from pprint import pprint
import copy
from pylab import *

class Bunch:
  def __init__(self, **kwds):
    self.__dict__.update(kwds)
  def dict(self):
    return self.__dict__

def main():
  e = Bunch()
  e.dataset="PSB"
  e.feature="PFH"
  e.data=None
  e.data = parse_log(e)

def parse_log(e):
  log_location_template = "../writeups/%s.out"
  with open(log_location_template%e.feature.lower()) as log:
    lines = log.readlines()

  e.trials = []
  i = 0
  while i < len(lines):
    l = lines[i]

    # Get number of models
    if l.find('[training]') > -1:
      # get the integer in the line above this one
      highest_model_ind = int(re.search('finished model (\d+)', lines[i-1]).group(1))
      e.num_models = highest_model_ind+1

    # Model test output
    if l.find('[test') > -1:
      trial = Bunch() 
      trial.trial_num = int(re.search('test (\d+)', l).group(1))
      trial.model_ind = int(re.search('scanned model (\d+)', lines[i+1]).group(1))
      trial.model_at_rank = []
      for j in range(4):
        # get the first number on the line
        trial.model_at_rank.append(int(re.search('^(\d+)', lines[i+3+j]).group(1)))
      trial.detector_guess = int(re.search('detector guessed (\d+)', lines[i+3+4]).group(1))
      e.trials.append(trial)
    e.num_trials = len(e.trials)
    
    # Overview output
    if l.find('[overview]') > -1:
      m = re.search('^(\d+)', lines[i+1]) # first number
      num_correct = int(m.group(1))

    # Precision-Recall output
    if l.find('[precision-recall]') > -1:
      e.precision = []
      e.recall = []
      j = 1
      while True:
        if lines[i+j].find('[classifier stats]') > -1:
          break
        numbers = lines[i+j].split()
        e.precision.append(float(numbers[0]))
        e.recall.append(float(numbers[1]))
        j += 1
      e.ap = VOCap(e.recall,e.precision)

    if l.find('[classifier stats]') > -1:
      # get the last numbers in the next two lines
      e.avg_rank = float(lines[i+1].split()[-1])
      e.auh = float(lines[i+2].split()[-1])
      # NOTE: dividing by the total possible area to get a [0,1] number
      e.auh /= e.num_models*e.num_trials

    if l.find('[confusion matrix]') > -1:
      # get the num_models lines following this one
      data = lines[i+1:i+1+e.num_models]
      data = [line.strip() for line in data]
      e.confusion_matrix = matrix(';'.join(data))

    if l.find('[timing]') > -1:
      print("TIMING HERE")

    i += 1

  model_name_location = "../writeups/model_names.txt"
  with open(model_name_location) as f:
    lines = f.readlines()
  e.model_names = [line.strip() for line in lines]

  pprint(e.trials[0].dict())
  print("Num correct: %s"%num_correct)
  print("AP: %s"%e.ap)
  print("AUH: %s"%e.auh)
  print("Average rank: %s"%e.avg_rank)
  plot_pr([e])
  plot_confusion_matrix(e)
  plot_rank_histogram(e)

def rank_histogram_data(e):
  # Go through and count the number of times the right model was rank-1,
  # rank-2, etc.
  # construct matrix of all the model_at_ranks
  all_model_at_rank = zeros((e.num_trials,4))
  correct_model = zeros((e.num_trials,1))
  for i,trial in enumerate(e.trials):
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

def plot_rank_histogram(e):
  """Takes an evaluation and outputs its rank histogram to file."""
  clf()
  counts = rank_histogram_data(e) 
  counts_r = 1.*counts/sum(counts)
  ind = arange(5)
  width = 0.9
  bar(ind,cumsum(counts_r),width=.9,color='black')
  for i,count in enumerate(cumsum(counts_r)):
    text(i+width/2.-.1, count*1.-.04, "%.3f"%count, color='white')
  ranks = ['1','2','3','4','5+']
  xticks(ind+width/2., ranks) 
  xlabel('Number of top results within which the correct model is fetched')
  #ylabel('Proportion')
  title('Area under the rank histogram is %.3f'%e.auh)

  filename = "%s_rankhist.png"%e.feature
  savefig(filename)

def plot_confusion_matrix(e):
  """Takes an evaluation and outputs its confusion matrix to file."""
  # code from http://stackoverflow.com/questions/2897826/confusion-matrix-with-number-of-classified-misclassified-instances-on-it-python
  conf_arr = e.confusion_matrix.tolist()

  norm_conf = []
  for i in conf_arr:
    a = 0
    tmp_arr = []
    a = sum(i,0)
    for j in i:
      tmp_arr.append(float(j)/float(a))
    norm_conf.append(tmp_arr)

  clf()
  fig = figure()
  ax = fig.add_subplot(111)
  res = ax.imshow(array(norm_conf), cmap=cm.gray_r, interpolation='nearest')
  for i, cas in enumerate(norm_conf):
    for j, c in enumerate(cas):
      if c>0:
        # TODO: adjust the location of the text
        if c <= 0.5:
          text(j-.2, i+.2, "%.2f"%c, color='black', fontsize=12)
        else:
          text(j-.2, i+.2, "%.2f"%c, color='white', fontsize=12)
  cb = fig.colorbar(res)
  xticks(arange(0,e.num_models), e.model_names, rotation=30, size='small')
  yticks(arange(0,e.num_models), e.model_names, size='small')

  filename = "%s_confmat.png"%e.feature
  savefig(filename)

def plot_pr(evaluations):
  """Takes a list of evaluations and plots their PR curves."""
  clf()
  linestyles = ['-', '--', ':']
  lines = []
  for i,e in enumerate(evaluations):
    label = "%s: %.3f"%(e.feature,e.ap)
    lstyle = linestyles[mod(i,len(linestyles))]
    lines.append(plot(e.recall,e.precision,lstyle,label=label))
  legend()
  xlabel('Recall')
  ylabel('Precision')
  grid(True)

  filename = "%s_pr.png"%('-'.join([e.feature for e in evaluations]))
  savefig(filename)

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

if __name__ == '__main__':
  main()
