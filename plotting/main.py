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

  trials = []
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
      trial = {}
      trial['model_num'] = int(re.search('test (\d+)', l).group(1))
      trial['model_at_rank'] = []
      for j in range(4):
        # get the first number on the line
        trial['model_at_rank'].append(int(re.search('^(\d+)', lines[i+3+j]).group(1)))
      trial['detector_guess'] = int(re.search('detector guessed (\d+)', lines[i+3+4]).group(1))
      trials.append(trial)
    
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

    if l.find('[confusion matrix]') > -1:
      # get the num_models lines following this one
      data = lines[i+1:i+1+e.num_models]
      data = [line.strip() for line in data]
      e.confusion_matrix = matrix(';'.join(data))

    if l.find('[timing]') > -1:
      print("TIMING HERE")

    i += 1

  # TODO: load actual model_names
  model_name_location = "../writeups/model_names.txt"
  with open(model_name_location) as f:
    lines = f.readlines()
  e.model_names = [line.strip() for line in lines]

  #pprint(trials)
  print("Num correct: %s"%num_correct)
  print("AP: %s"%e.ap)
  print("AUH: %s"%e.auh)
  print("Average rank: %s"%e.avg_rank)
  #plot_pr([e])
  plot_confusion_matrix(e)

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
  xticks(arange(0,e.num_models), e.model_names)
  yticks(arange(0,e.num_models), e.model_names)

  filename = "%s_confmat.png"%e.feature
  savefig(filename)

def plot_pr(evaluations):
  """Takes a list of evaluations and plots their PR curves."""
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
