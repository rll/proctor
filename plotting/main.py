#!/usr/bin/env python
"""
Processes all data output by Proctor to output all plots and Latex-source
tables that we could need for publication.
"""
import re
from pprint import pprint
import copy
import pylab 

class Bunch:
  def __init__(self, **kwds):
    self.__dict__.update(kwds)



def main():
  evaluation = Bunch(
      dataset="PSB",
      feature="PFH",
      data=None)
  evaluation.data = parse_log(evaluation)

def parse_log(e):
  log_location_template = "../writeups/%s.out"
  with open(log_location_template%e.feature.lower()) as log:
    lines = log.readlines()

  trials = []
  i = 0
  while i < len(lines):
    l = lines[i]
    #print(l)
    
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

    if l.find('[classifier stats]') > -1:
      print("CLASSIFIER STATS HERE")

    if l.find('[confusion matrix]') > -1:
      print("CM HERE")

    if l.find('[timing]') > -1:
      print("TIMING HERE")

    i += 1

  #pprint(trials)
  print("Num correct: %s"%num_correct)
  e2 = copy.deepcopy(e)
  e2.precision = [x+0.05 for x in e.precision]
  plot_pr([e,e2])

def plot_pr(evaluations):
  """Takes a list of evaluations and plots their PR curves."""
  lines = []
  for e in evaluations:
    lines.append(pylab.plot(e.recall,e.precision,label=e.feature))
  pylab.legend()
  pylab.xlabel('Recall')
  pylab.ylabel('Precision')
  pylab.grid(True)
  pylab.savefig('temp.png')
    
if __name__ == '__main__':
  main()
