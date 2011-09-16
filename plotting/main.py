#!/usr/bin/env python
"""
Processes all data output by Proctor to output all plots and Latex-source
tables that we could need for publication.
"""
import re
import os
from pprint import pprint
import copy
from pylab import *

from evaluation import Evaluation

class EvalSet:
  preamble="""\documentclass[10pt]{article}
\usepackage[usenames]{color} %used for font color
\usepackage{amssymb} %maths
\usepackage{amsmath} %maths
\usepackage[utf8]{inputenc} %useful to type directly diacritic characters
\usepackage{multirow}
\usepackage{subfig}
\\begin{document}

"""  

  def plot_pr(self,features=None):
    """Takes a list of features and plots their PR curves on the same plot."""
    evals,feat_names = self.process_features(features)
    clf()
    linestyles = ['-', '--', ':']
    for i,f in enumerate(features):
      e = self.evals[f]
      label = "%s: %.3f"%(e.feature,e.ap)
      lstyle = linestyles[mod(i,len(linestyles))]
      plot(e.recall,e.precision,lstyle,label=label)
    legend(loc='lower left')
    xlabel('Recall')
    ylabel('Precision')
    grid(True)

    self.savefig_wrap(features,'pr')

  def print_info(self, feature):
    e = self.evals[feature]
    print("Feature: %s"%feature)
    print("  Num correct: %s"%e.num_correct)
    print("  Accuracy: %s"%e.accuracy)
    print("  AP: %s"%e.ap)
    print("  Average rank: %s"%e.avg_rank)
    print("  AUH: %s"%e.auh)

  def plot_rank_histogram(self, features=None):
    evals,feat_names = self.process_features(features)
    colors = ['blue','green','red','cyan']
    clf()
    width = .9/len(features)
    ind = arange(5)
    for i,feature in enumerate(features):
      e = self.evals[feature]
      counts = e.rank_histogram_data() 
      counts_r = 1.*counts/sum(counts)
      color = colors[mod(i, len(colors))]
      # only print the value if doing one feature
      if len(features)==1:
        color = 'black'
        for j,count in enumerate(cumsum(counts_r)):
          text(j+width/2.-.1, count*1.-.04, "%.3f"%count, color='white')
      bar(ind+i*width,cumsum(counts_r),color=color,width=width,label=feature)
    ranks = ['1','2','3','4','5+']
    xticks(ind+width/2., ranks) 
    xlabel('Number of top results within which the correct model is fetched')
    title('Area under the rank histogram is %.3f.'%e.auh)
    if len(features)>1:
      legend(loc='upper left')
    self.savefig_wrap(features,'rankhist')

  def plot_confusion_matrix(self,feature):
    """Takes an evaluation and outputs its confusion matrix to file."""
    # code from http://stackoverflow.com/questions/2897826/confusion-matrix-with-number-of-classified-misclassified-instances-on-it-python
    e = self.evals[feature]
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
    xticks(arange(0, self.num_models), self.model_names, rotation=30, size='small')
    yticks(arange(0, self.num_models), self.model_names, size='small')

    self.savefig_wrap([feature],'confmat')

  def savefig_wrap(self, features, suffix):
    """Saves the plot to a canonical name."""
    evals,feat_names = self.process_features(features)
    filename = self.plot_location+"%s_%s.png"%(feat_names,suffix)
    savefig(filename)

  def print_comparison_table(self,features=None,pdf=True):
    """
    Outputs a .tex file containing the table of results for the given features.
    Optionally, also generates a .pdf containing this with a minimal preamble.
    """
    evals,feat_names = self.process_features(features)
    tex_filename = self.table_tex_filename_template%feat_names
    with open(tex_filename,'w') as f:
      print_with_max = lambda numbers: ' & '.join(["\\bf %.2f"%x if x==max(numbers) else "%.2f"%x for x in numbers])
      accuracies = [100*e.accuracy for e in evals]
      aps = [e.ap for e in evals]
      avg_ranks = [e.avg_rank for e in evals]
      auhs = [e.auh for e in evals]
      table = '\n'.join([
        '\\begin{tabular}{ | l || %s | }'%' | '.join(['l' for e in evals]),
        '\hline',
        'Metric & '+' & '.join([e.nice_name for e in evals])+' \\\\', 
        '\hline',
        ' \% Correct & '+print_with_max(accuracies)+' \\\\', 
        'AP & '+print_with_max(aps)+' \\\\', 
        'Avg. Rank & '+print_with_max(avg_ranks)+' \\\\', 
        'AUH & '+print_with_max(auhs)+' \\\\', 
        '\hline',
        '\end{tabular}'])
      f.write(table)
    if pdf:
      self.preview_pdf(tex_filename)

  def preview_pdf(self,tex_filename):
    pdf_tex_filename = tex_filename[:-4]+'_preview.tex'
    with open(pdf_tex_filename,'w') as f:
      f.write(self.preamble)
      f.write('\input{%s}'%os.path.basename(tex_filename))
      f.write('\n\end{document}')
    pdf_filename = os.path.basename(pdf_tex_filename)[:-4]+'.pdf'
    cmds = [
        "cd %s"%self.table_location,
        "make %s"%pdf_filename,
        "mv %s %s1"%(pdf_filename,pdf_filename),
        "make clean",
        "mv %s1 %s"%(pdf_filename,pdf_filename)]
    os.system(' && '.join(cmds))

  def generate_subfig(self,features=None,pdf=True):
    """
    Outputs a .tex file containing a subfigure pulling together all the plots
    for the given features.
    Optionally, also generates a .pdf containing this with a minimal preamble.
    """
    evals,feat_names = self.process_features(features)


  def process_features(self,features=None):
    """
    Returns the eval data and name string for the set of features passed in.
    If nothing is passed in, uses all features of this eval_set.
    """
    if not features:
      features = self.features
    evals = [self.evals[feature] for feature in features]
    feat_names = '-'.join([f for f in features])
    return (evals,feat_names)


def main():
  """
  Goes through different features, outputting their evaluations, as well as
  a combined PR curve evaluation.
  """
  eval_set = EvalSet()
  eval_set.features = ['PFH','FPFH','SHOT','SPIN_IMAGE']

  # First, load the common model names
  model_name_location = "../writeups/model_names.txt"
  with open(model_name_location) as f:
    lines = f.readlines()
  eval_set.model_names = [line.strip() for line in lines]
  eval_set.num_models = len(eval_set.model_names)
  eval_set.dataset = "PSB"
  eval_set.log_location_template = "../writeups/%s.out"
  eval_set.plot_location = "../writeups/figures/"
  if not os.path.exists(eval_set.plot_location):
    os.makedirs(eval_set.plot_location)
  eval_set.table_location = "../writeups/results/"
  if not os.path.exists(eval_set.table_location):
    os.makedirs(eval_set.table_location)
  eval_set.table_tex_filename_template = eval_set.table_location+'%s'+"_table.tex"


  # Parse the output log data for all the evaluations
  eval_set.evals = {}
  for feature in eval_set.features:
    eval_set.evals[feature] = Evaluation(eval_set, feature)
    eval_set.print_info(feature)
    #eval_set.plot_confusion_matrix(feature)
    #eval_set.plot_rank_histogram([feature])
    #eval_set.plot_pr([feature])

  # Print feature comparison table and output PR plot
  eval_set.print_comparison_table()
  #eval_set.plot_rank_histogram()
  #eval_set.plot_pr()

  # Generate consolidation latex figure
  eval_set.generate_subfig()

if __name__ == '__main__':
  main()
