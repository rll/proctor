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
  preamble=r"""
\documentclass[10pt]{article}
\usepackage[usenames]{color} %used for font color
\usepackage{amssymb} %maths
\usepackage{amsmath} %maths
\usepackage[utf8]{inputenc} %useful to type directly diacritic characters
\usepackage{multirow}

\usepackage{color}
\usepackage{graphicx}
\usepackage{subfig}
\usepackage{array}
\usepackage{url}

\begin{document}

"""  

  def plot_pr(self,features=None):
    """Takes a list of features and plots their PR curves on the same plot."""
    evals,feat_names = self.process_features(features)
    clf()
    linestyles = ['-', '--', ':']
    for i,e in enumerate(evals):
      label = "%s: %.3f"%(e.nice_name,e.ap)
      lstyle = linestyles[mod(i,len(linestyles))]
      plot(e.recall,e.precision,lstyle,label=label,linewidth=3)
    legend(loc='lower left')
    xlabel('Recall',size=16)
    ylabel('Precision',size=16)
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
    pprint(e.time)

  def plot_timing(self, features=None):
    evals,feat_names = self.process_features(features)
    names = [e.nice_name for e in evals]
    colors = ['blue','green','red','cyan']
    width = 0.9
    ind = arange(len(evals))

    data_sources = ['train_total','test_total']
    for data_source in data_sources:
      data = [e.time[data_source] for e in evals]
      ax = self.prep_bar((8,4))
      subplots_adjust(bottom=0.16)
      barh(ind,data,color='black')
      for j,d in enumerate(data):
        text(d+max(data)*0.01, j+0.4, "%.2f"%d)
      
      yticks(ind+0.5,names,size=16)
      xlabel('Time in seconds',size=16)
      ax.xaxis.grid(True)
      self.savefig_wrap(features,data_source)

  def plot_timing_stacked(self, features=None):
    evals,feat_names = self.process_features(features)
    names = [e.nice_name for e in evals]
    colors = ['blue','green','red','cyan']
    width = 0.9
    ind = arange(len(evals))

    self.prep_bar((8,4))
    feat_times = [e.time['test_features'] for e in evals]
    print(feat_times)
    p1 = barh(ind,feat_times,color=colors[0],label='Feature computation')

    vote_times = [e.time['voting'] for e in evals]
    print(vote_times)
    p2 = barh(ind,vote_times,left=feat_times,color=colors[1],label='Voting')

    align_times = [e.time['alignment_inferred'] for e in evals]
    print(align_times)
    p3 = barh(ind,align_times,left=add(feat_times,vote_times),color=colors[2],label='Alignment')

    icp_times = [e.time['ICP'] for e in evals]
    print(icp_times)
    p4 = barh(ind,icp_times, left=add(add(feat_times,vote_times),align_times),color=colors[3],label='ICP')

    yticks(ind+0.5,names)
    xlabel('Time in seconds')
    legend(loc='lower right')
    self.savefig_wrap(features,'timing_test')

  def prep_bar(self,size=None):
    fig = figure(1)
    fig.clf()
    if size:
      fig.set_size_inches(size)
    ax = fig.add_subplot('111')
    self.turn_off_up_right_axes(ax)
    return ax

  def plot_timing_panel(self, features=None):
    evals,feat_names = self.process_features(features)
    names = [e.nice_name for e in evals]
    colors = ['blue','green','red','cyan']
    width = 0.9
    ind = arange(len(evals))

    clf()
    fig, ((ax1, ax2, ax3, ax4)) = subplots(1, 4, figsize=(16,4), dpi=150, sharex=False, sharey=True)
    subplots_adjust(bottom=0.16,left=0.05,right=0.95)
    self.turn_off_up_right_axes(ax1)
    self.turn_off_up_right_axes(ax2)
    self.turn_off_up_right_axes(ax3)
    self.turn_off_up_right_axes(ax4)
    setp(ax2.get_yticklabels(), visible=False)
    setp(ax3.get_yticklabels(), visible=False)
    setp(ax4.get_yticklabels(), visible=False)
    
    axs = [ax1,ax2,ax3,ax4]
    data_sources = ['test_features','voting','alignment_inferred','ICP']
    xlabels = ['Feature Description', 'Voting', 'Initial Alignment', 'ICP']

    for ax,data_source,xlabel in zip(axs,data_sources,xlabels):
      times = [e.time[data_source] for e in evals]
      ax.barh(ind,times,color='black')
      for j,d in enumerate(times):
        ax.text(d+max(times)*0.02, j+0.35, "%.2f"%d)
      ax.xaxis.grid(True)
      ax.set_xlabel('Time (sec)')
      setp(ax.get_xticklabels(),rotation=30)
      ax.set_title(xlabel)
      yticks(ind+0.5,names)

    self.savefig_wrap(features,'timing_test_panel')

  def turn_off_up_right_axes(self,ax):
    for loc, spine in ax.spines.iteritems():
      if loc in ['left','bottom']:
        #spine.set_position(('outward',10)) # outward by 10 points
        None
      elif loc in ['right','top']:
        spine.set_color('none') # don't draw spine
      else:
        raise ValueError('unknown spine location: %s'%loc)
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    return ax

  def plot_rank_histogram(self, features=None):
    evals,feat_names = self.process_features(features)
    colors = ['blue','green','red','cyan']
    rc('text', usetex=True)
    clf()
    width = .9/len(evals)
    ind = arange(4)
    for i,e in enumerate(evals):
      counts = e.rank_histogram_data() 
      counts = counts[:4] # not using the catchall column
      counts_r = 1.*counts/(e.num_trials)
      color = colors[mod(i, len(colors))]
      # only print the value if doing one feature
      label = '%s: %.3f'%(e.nice_name, e.auh)
      if len(evals)==1:
        label = '%.3f'%e.auh
        color = 'black'
        for j,count in enumerate(cumsum(counts_r)):
          text(j+width/2.-.1, count*1.-.04, "%.2f"%count, color='white')
      bar(ind+i*width,cumsum(counts_r),color=color,width=width,label=label)
    ranks = [r'$\leq %s$'%x for x in [1,2,3,4]]
    xticks(ind+0.5, ranks,size=16)
    yticks(linspace(0,1,5),size=16)
    xlabel('Correct result fetched within K hits ',size=16)
    ylabel('Proportion of trials',size=16)
    legend(loc='upper left')
    self.savefig_wrap(features,'rankhist')
    rc('text', usetex=False)

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
    #cb = fig.colorbar(res)
    xticks(arange(0, self.num_models), self.model_names, rotation=30)
    yticks(arange(0, self.num_models), self.model_names, size=16)

    self.savefig_wrap([feature],'confmat')

  def savefig_wrap(self, features, suffix,fig=None):
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
      print_with_min = lambda numbers: ' & '.join(["\\bf %.2f"%x if x==min(numbers) else "%.2f"%x for x in numbers])
      accuracies = [100*e.accuracy for e in evals]
      aps = [e.ap for e in evals]
      avg_ranks = [e.avg_rank for e in evals]
      auhs = [e.auh for e in evals]
      table = '\n'.join([
        '\\begin{tabular}{ | l || %s | }'%' | '.join(['c' for e in evals]),
        '\hline',
        'Metric & '+' & '.join([e.nice_name for e in evals])+' \\\\', 
        '\hline',
        ' \% Correct & '+print_with_max(accuracies)+' \\\\', 
        'AP & '+print_with_max(aps)+' \\\\', 
        'Avg. Rank & '+print_with_min(avg_ranks)+' \\\\', 
        'AUH & '+print_with_max(auhs)+' \\\\', 
        '\hline',
        '\end{tabular}'])
      f.write(table)
    if pdf:
      self.preview_pdf(tex_filename)

  def generate_subfig(self,features=None,pdf=True):
    """
    Outputs a .tex file containing a subfigure pulling together all the plots
    for the given features.
    Optionally, also generates a .pdf containing this with a minimal preamble.
    """
    # TODO: make sure this matches the dashboard that's in the paper. Right now
    # it does not.
    evals,feat_names = self.process_features(features)
    col_widths = (0.08, 0.35, 0.35)
    subfig = r"""
\begin{table*}
\centering
\begin{tabular}{m{%s\textwidth} m{%s\textwidth} m{%s\textwidth}}
  & \begin{center} Confusion Matrix \end{center} & \begin{center} Cumulative Rank Histogram \end{center} \\
"""%col_widths
    subfig += '\n'.join([
r'  %(name)s & \includegraphics[width=%(width1)s\textwidth,clip=true]{../figures/%(dataset)s/%(feature)s_confmat.png} & \includegraphics[width=%(width2)s\textwidth,clip=true]{../figures/%(dataset)s/%(feature)s_rankhist.png} \\' % dict(name=e.nice_name, dataset=self.dataset, feature = e.feature, width1 = col_widths[1], width2 = col_widths[2]) for e in evals])
    subfig += r"""
\end{tabular}
\caption{A table arranging images}
\label{tab:gt}
\end{table*}"""
# TODO change the caption
    tex_filename = self.features_tex_filename_template%feat_names
    with open(tex_filename,'w') as f:
      f.write(subfig)
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

  def process_features(self,features=None):
    """
    Returns the eval data and name string for the set of features passed in.
    If nothing is passed in, uses all features of this e_set.
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
  do_plot = True 
  #do_plot = False 

  # Set the basic things about this run
  # TODO: accept these from the command line
  e_set = EvalSet()
  dataset = "PSB"
  if dataset == "WGDB":
    e_set.dataset = "WGDB"
    e_set.dataset_full = "Willow Garage Grasping Dataset"
  else:
    e_set.dataset = "PSB"
    e_set.dataset_full = "Princeton Shape Benchmark"
  e_set.features = ['PFH','FPFH','SHOT','SPIN_IMAGE']

  # Load the common model names
  model_name_location = "../results/%s/model_names.txt"%e_set.dataset.lower()
  with open(model_name_location) as f:
    lines = f.readlines()
  e_set.model_names = [line.strip() for line in lines]
  e_set.num_models = len(e_set.model_names)

  # Set the paths to write out to, creating directories if needed
  e_set.log_location_template = "../results/%s"%e_set.dataset.lower()+"/%s.out"
  e_set.plot_location = "../writeups/figures/%s/"%e_set.dataset
  if not os.path.exists(e_set.plot_location):
    os.makedirs(e_set.plot_location)
  e_set.table_location = "../writeups/results/"
  if not os.path.exists(e_set.table_location):
    os.makedirs(e_set.table_location)
  # NOTE: We need the paths from these files to be the same as from the actual
  # writeup, so we can't add an additional folder. Hence the name of the
  # dataset as part of the filename.
  e_set.table_tex_filename_template = e_set.table_location+e_set.dataset+'_%s_table.tex'
  e_set.features_tex_filename_template = e_set.table_location+e_set.dataset+'_%s_features.tex'

  # Parse the output log data
  e_set.evals = {}
  for feature in e_set.features:
    e_set.evals[feature] = Evaluation(e_set, feature)
    e_set.print_info(feature)
    if do_plot:
      e_set.plot_confusion_matrix(feature)
      e_set.plot_rank_histogram([feature])
      e_set.plot_pr([feature])

  # Print feature comparison table and output PR plot
  if do_plot:
    e_set.plot_rank_histogram()
    e_set.plot_pr()
    e_set.plot_timing()
  e_set.plot_timing_panel()

  # Generate feature comparison table and latex figure
  e_set.print_comparison_table()
  e_set.generate_subfig()

if __name__ == '__main__':
  main()
