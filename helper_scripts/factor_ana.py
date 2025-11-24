#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GNSS-IMU Fusion Factor Analysis Tool

This script analyzes the factor graph growth and optimization performance
based on CSV output from the GNSS-IMU fusion system.

Author: Generated for batch_board system
Usage: python3 factor_ana.py --csv /path/to/state_file.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
import glob
import os
from datetime import datetime

class FactorAnalyzer:
    def __init__(self, csv_file_path=None, output_dir=None):
        """
        Initialize the Factor Analyzer
        
        Args:
            csv_file_path: Path to the state CSV file
            output_dir: Directory to save analysis plots
        """
        self.csv_file = csv_file_path
        # 修改默认输出目录为当前工作目录
        self.output_dir = output_dir or os.getcwd()
        self.data = None
        self.setup_plotting()
        
    def setup_plotting(self):
        """Setup matplotlib and seaborn plotting parameters"""
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.titlesize'] = 14
        plt.rcParams['axes.labelsize'] = 12
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        
        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
    def find_latest_csv(self, csv_dir=None):
        """Find the latest state CSV file in the given directory"""
        # 如果没有指定目录，优先在当前目录查找，然后在默认输出目录查找
        if csv_dir is None:
            search_dirs = [
                os.getcwd(),  # 当前工作目录
                "/home/jay/batch_board/output"  # 默认输出目录作为备选
            ]
        else:
            search_dirs = [csv_dir]
        
        csv_files = []
        for search_dir in search_dirs:
            pattern = os.path.join(search_dir, "gnss_imu_state_*.csv")
            found_files = glob.glob(pattern)
            csv_files.extend(found_files)
            if found_files:
                print(f"Found CSV files in directory: {search_dir}")
                break
        
        if not csv_files:
            searched_dirs = ', '.join(search_dirs)
            raise FileNotFoundError(f"No state CSV files found in directories: {searched_dirs}")
            
        # Sort by modification time, get the latest
        latest_file = max(csv_files, key=os.path.getmtime)
        print(f"Found latest CSV file: {latest_file}")
        return latest_file
        
    def load_data(self, csv_file=None):
        """Load and preprocess CSV data"""
        if csv_file:
            self.csv_file = csv_file
        elif not self.csv_file:
            self.csv_file = self.find_latest_csv()
            
        print(f"Loading data from: {self.csv_file}")
        
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.data)} records")
            
            # Convert timestamp to datetime for better plotting
            self.data['datetime'] = pd.to_datetime(self.data['timestamp'], unit='s')
            
            # Calculate relative time from start (in minutes)
            start_time = self.data['timestamp'].min()
            self.data['relative_time_min'] = (self.data['timestamp'] - start_time) / 60.0
            
            # Calculate factor growth rates
            factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                          'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
            
            for col in factor_cols:
                if col in self.data.columns:
                    self.data[f'{col}_rate'] = self.data[col].diff() / self.data['relative_time_min'].diff()
                    
            # Calculate optimization efficiency metrics
            if 'optimization_time_ms' in self.data.columns and 'total_factors' in self.data.columns:
                self.data['time_per_factor'] = self.data['optimization_time_ms'] / self.data['total_factors']
                
            print("Data preprocessing completed")
            self.print_data_summary()
            
        except Exception as e:
            raise RuntimeError(f"Error loading CSV file: {e}")
            
    def print_data_summary(self):
        """Print summary statistics of the loaded data"""
        if self.data is None:
            print("No data loaded")
            return
            
        print("\n" + "="*50)
        print("DATA SUMMARY")
        print("="*50)
        print(f"Total records: {len(self.data)}")
        print(f"Time span: {self.data['relative_time_min'].max():.1f} minutes")
        print(f"Final keyframe count: {self.data['keyframe_count'].max()}")
        
        if 'total_factors' in self.data.columns:
            print(f"Final total factors: {self.data['total_factors'].max()}")
            
        # Factor breakdown
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        print("\nFactor breakdown (final counts):")
        for col in factor_cols:
            if col in self.data.columns:
                final_count = self.data[col].max()
                percentage = (final_count / self.data['total_factors'].max()) * 100 if self.data['total_factors'].max() > 0 else 0
                print(f"  {col.replace('total_', '').replace('_', ' ').title()}: {final_count} ({percentage:.1f}%)")
                
        print("="*50)
        
    def plot_factor_evolution(self):
        """Plot the evolution of different factor types over time"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Factor Graph Evolution Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Total factors over time
        ax1 = axes[0, 0]
        if 'total_factors' in self.data.columns:
            ax1.plot(self.data['relative_time_min'], self.data['total_factors'], 
                    'b-', linewidth=2, label='Total Factors')
            ax1.set_title('Total Factors Over Time')
            ax1.set_xlabel('Time (minutes)')
            ax1.set_ylabel('Number of Factors')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            
        # Plot 2: Factor type breakdown (individual lines with colors)
        ax2 = axes[0, 1]
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        # 定义颜色和标签的映射
        factor_colors = {
            'total_gnss_pos_factors': '#1f77b4',  # 蓝色
            'total_gnss_vel_factors': '#ff7f0e',  # 橙色  
            'total_imu_factors': '#2ca02c',       # 绿色
            'total_bias_factors': '#d62728',      # 红色
            'total_prior_factors': '#9467bd'      # 紫色
        }
        
        factor_labels = {
            'total_gnss_pos_factors': 'GNSS Position',
            'total_gnss_vel_factors': 'GNSS Velocity',
            'total_imu_factors': 'IMU Preintegration',
            'total_bias_factors': 'Bias Between',
            'total_prior_factors': 'Prior Factors'
        }
        
        valid_cols = [col for col in factor_cols if col in self.data.columns]
        if valid_cols:
            for col in valid_cols:
                ax2.plot(self.data['relative_time_min'], self.data[col], 
                        color=factor_colors.get(col, 'gray'),
                        linewidth=2.5, 
                        label=factor_labels.get(col, col.replace('total_', '').title()),
                        marker='o', 
                        markersize=3,
                        alpha=0.8)
            
            ax2.set_title('Individual Factor Types Over Time')
            ax2.set_xlabel('Time (minutes)')
            ax2.set_ylabel('Number of Factors')
            ax2.legend(loc='upper left', framealpha=0.9)
            ax2.grid(True, alpha=0.3)
            
            # 添加最终数量的文本标注
            for col in valid_cols:
                final_count = self.data[col].iloc[-1]
                final_time = self.data['relative_time_min'].iloc[-1]
                ax2.annotate(f'{int(final_count)}', 
                           xy=(final_time, final_count),
                           xytext=(5, 5), 
                           textcoords='offset points',
                           fontsize=9,
                           alpha=0.8,
                           color=factor_colors.get(col, 'gray'))
            
        # Plot 3: Keyframes vs Factors
        ax3 = axes[1, 0]
        if 'keyframe_count' in self.data.columns and 'total_factors' in self.data.columns:
            scatter = ax3.scatter(self.data['keyframe_count'], self.data['total_factors'], 
                                c=self.data['relative_time_min'], cmap='viridis', alpha=0.6, s=30)
            ax3.set_title('Factors vs Keyframes')
            ax3.set_xlabel('Keyframe Count')
            ax3.set_ylabel('Total Factors')
            ax3.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax3)
            cbar.set_label('Time (minutes)')
            
            # 添加趋势线
            if len(self.data) > 2:
                z = np.polyfit(self.data['keyframe_count'], self.data['total_factors'], 1)
                p = np.poly1d(z)
                ax3.plot(self.data['keyframe_count'], p(self.data['keyframe_count']), 
                        "r--", alpha=0.8, linewidth=2,
                        label=f'Trend: {z[0]:.1f} factors/keyframe')
                ax3.legend()
            
        # Plot 4: Factor增长速度 (每轮优化新增的因子数量)
        ax4 = axes[1, 1]
        
        # 计算每轮优化新增的因子数量
        factor_cols_for_growth = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                                 'total_imu_factors', 'total_bias_factors']
        
        valid_growth_cols = [col for col in factor_cols_for_growth if col in self.data.columns]
        
        if valid_growth_cols:
            # 创建柱状图显示每轮新增的因子
            x_positions = range(len(self.data))
            bar_width = 0.8
            bottom = np.zeros(len(self.data))
            
            growth_colors = {
                'total_gnss_pos_factors': '#1f77b4',
                'total_gnss_vel_factors': '#ff7f0e', 
                'total_imu_factors': '#2ca02c',
                'total_bias_factors': '#d62728'
            }
            
            growth_labels = {
                'total_gnss_pos_factors': 'GNSS Pos',
                'total_gnss_vel_factors': 'GNSS Vel',
                'total_imu_factors': 'IMU',
                'total_bias_factors': 'Bias'
            }
            
            for col in valid_growth_cols:
                # 计算每轮新增数量 (当前值 - 前一个值)
                factor_diff = self.data[col].diff().fillna(self.data[col].iloc[0])
                factor_diff = factor_diff.clip(lower=0)  # 确保非负值
                
                if factor_diff.sum() > 0:  # 只有当有增长时才绘制
                    ax4.bar(x_positions, factor_diff, 
                           bottom=bottom,
                           width=bar_width,
                           label=growth_labels.get(col, col.replace('total_', '').title()),
                           color=growth_colors.get(col, 'gray'),
                           alpha=0.8)
                    bottom += factor_diff
            
            ax4.set_title('Factors Added Per Optimization Round')
            ax4.set_xlabel('Optimization Round')
            ax4.set_ylabel('New Factors Added')
            ax4.legend(loc='upper right', framealpha=0.9)
            ax4.grid(True, alpha=0.3, axis='y')
            
            # 设置x轴标签，每隔几个显示一次避免拥挤
            if len(self.data) > 20:
                step = len(self.data) // 10
                ax4.set_xticks(range(0, len(self.data), step))
                ax4.set_xticklabels(range(0, len(self.data), step))
            
            # 添加总数统计
            total_new = bottom.sum()
            ax4.text(0.02, 0.98, f'Total new factors: {int(total_new)}', 
                    transform=ax4.transAxes, 
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'factor_evolution.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Factor evolution plot saved: {output_path}")
        plt.show()
        
    def plot_optimization_performance(self):
        """Plot optimization performance metrics"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Optimization Performance Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Optimization time over iterations
        ax1 = axes[0, 0]
        if 'optimization_time_ms' in self.data.columns:
            ax1.plot(self.data['seq'], self.data['optimization_time_ms'], 
                    'r-', linewidth=2, alpha=0.7)
            ax1.set_title('Optimization Time per Iteration')
            ax1.set_xlabel('Iteration')
            ax1.set_ylabel('Time (ms)')
            ax1.grid(True, alpha=0.3)
            
            # Add trend line
            if len(self.data) > 1:
                z = np.polyfit(self.data['seq'], self.data['optimization_time_ms'], 1)
                p = np.poly1d(z)
                ax1.plot(self.data['seq'], p(self.data['seq']), "--", alpha=0.8, color='darkred', 
                        label=f'Trend (slope: {z[0]:.3f} ms/iter)')
                ax1.legend()
        
        # Plot 2: Time per factor
        ax2 = axes[0, 1]
        if 'time_per_factor' in self.data.columns:
            valid_data = self.data['time_per_factor'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_data) > 0:
                ax2.plot(self.data.loc[valid_data.index, 'relative_time_min'], valid_data, 
                        'g-', linewidth=2, alpha=0.7)
                ax2.set_title('Optimization Efficiency (Time per Factor)')
                ax2.set_xlabel('Time (minutes)')
                ax2.set_ylabel('ms per Factor')
                ax2.grid(True, alpha=0.3)
        
        # Plot 3: Graph size vs optimization time
        ax3 = axes[1, 0]
        if 'graph_size' in self.data.columns and 'optimization_time_ms' in self.data.columns:
            scatter = ax3.scatter(self.data['graph_size'], self.data['optimization_time_ms'], 
                                c=self.data['relative_time_min'], cmap='plasma', alpha=0.6)
            ax3.set_title('Graph Size vs Optimization Time')
            ax3.set_xlabel('Graph Size (factors per iteration)')
            ax3.set_ylabel('Optimization Time (ms)')
            ax3.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax3)
            cbar.set_label('Time (minutes)')
        
        # Plot 4: High-frequency pose output rate
        ax4 = axes[1, 1]
        if 'high_freq_pose_count' in self.data.columns:
            # Calculate pose rate
            time_diff = self.data['relative_time_min'].diff()
            pose_rate = self.data['high_freq_pose_count'] / (time_diff * 60)  # poses per second
            valid_rates = pose_rate.replace([np.inf, -np.inf], np.nan).dropna()
            
            if len(valid_rates) > 1:
                ax4.plot(self.data.loc[valid_rates.index, 'relative_time_min'], valid_rates, 
                        'purple', linewidth=2, alpha=0.7)
                ax4.set_title('High-Frequency Pose Output Rate')
                ax4.set_xlabel('Time (minutes)')
                ax4.set_ylabel('Poses per Second')
                ax4.grid(True, alpha=0.3)
                
                # Add average line
                avg_rate = valid_rates.mean()
                ax4.axhline(y=avg_rate, color='red', linestyle='--', alpha=0.8, 
                           label=f'Average: {avg_rate:.1f} Hz')
                ax4.legend()
        
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'optimization_performance.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Optimization performance plot saved: {output_path}")
        plt.show()
        
    def plot_memory_analysis(self):
        """Plot memory usage analysis"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        memory_cols = ['virtual_memory_mb', 'physical_memory_mb', 'peak_memory_mb']
        available_memory_cols = [col for col in memory_cols if col in self.data.columns]
        
        if not available_memory_cols:
            print("No memory data available for analysis")
            return
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Memory Usage Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Memory usage over time
        ax1 = axes[0, 0]
        for col in available_memory_cols:
            ax1.plot(self.data['relative_time_min'], self.data[col], 
                    linewidth=2, label=col.replace('_', ' ').title(), alpha=0.8)
        ax1.set_title('Memory Usage Over Time')
        ax1.set_xlabel('Time (minutes)')
        ax1.set_ylabel('Memory (MB)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Memory vs factors
        ax2 = axes[0, 1]
        if 'physical_memory_mb' in self.data.columns and 'total_factors' in self.data.columns:
            scatter = ax2.scatter(self.data['total_factors'], self.data['physical_memory_mb'], 
                                c=self.data['relative_time_min'], cmap='coolwarm', alpha=0.6)
            ax2.set_title('Memory vs Total Factors')
            ax2.set_xlabel('Total Factors')
            ax2.set_ylabel('Physical Memory (MB)')
            ax2.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax2)
            cbar.set_label('Time (minutes)')
        
        # Plot 3: Memory growth rate
        ax3 = axes[1, 0]
        if 'memory_growth_mb' in self.data.columns:
            valid_growth = self.data['memory_growth_mb'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_growth) > 1:
                ax3.plot(self.data.loc[valid_growth.index, 'relative_time_min'], valid_growth, 
                        'orange', linewidth=2, alpha=0.7)
                ax3.set_title('Memory Growth Rate')
                ax3.set_xlabel('Time (minutes)')
                ax3.set_ylabel('Memory Growth (MB)')
                ax3.grid(True, alpha=0.3)
                ax3.axhline(y=0, color='red', linestyle='--', alpha=0.5)
        
        # Plot 4: Memory efficiency
        ax4 = axes[1, 1]
        if 'physical_memory_mb' in self.data.columns and 'total_factors' in self.data.columns:
            memory_per_factor = self.data['physical_memory_mb'] / self.data['total_factors']
            valid_efficiency = memory_per_factor.replace([np.inf, -np.inf], np.nan).dropna()
            
            if len(valid_efficiency) > 1:
                ax4.plot(self.data.loc[valid_efficiency.index, 'relative_time_min'], 
                        valid_efficiency, 'brown', linewidth=2, alpha=0.7)
                ax4.set_title('Memory Efficiency (MB per Factor)')
                ax4.set_xlabel('Time (minutes)')
                ax4.set_ylabel('MB per Factor')
                ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'memory_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Memory analysis plot saved: {output_path}")
        plt.show()
        
    def generate_summary_report(self):
        """Generate a comprehensive summary report"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        report = []
        report.append("GNSS-IMU FUSION FACTOR ANALYSIS REPORT")
        report.append("=" * 50)
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Data file: {os.path.basename(self.csv_file)}")
        report.append("")
        
        # Basic statistics
        report.append("BASIC STATISTICS:")
        report.append(f"  Total optimization iterations: {len(self.data)}")
        report.append(f"  Total runtime: {self.data['relative_time_min'].max():.1f} minutes")
        report.append(f"  Final keyframe count: {self.data['keyframe_count'].max()}")
        
        if 'total_factors' in self.data.columns:
            report.append(f"  Final total factors: {self.data['total_factors'].max()}")
            avg_factors_per_keyframe = self.data['total_factors'].max() / self.data['keyframe_count'].max()
            report.append(f"  Average factors per keyframe: {avg_factors_per_keyframe:.1f}")
        
        # Performance statistics
        report.append("")
        report.append("PERFORMANCE STATISTICS:")
        if 'optimization_time_ms' in self.data.columns:
            report.append(f"  Average optimization time: {self.data['optimization_time_ms'].mean():.2f} ms")
            report.append(f"  Max optimization time: {self.data['optimization_time_ms'].max():.2f} ms")
            report.append(f"  Min optimization time: {self.data['optimization_time_ms'].min():.2f} ms")
        
        if 'time_per_factor' in self.data.columns:
            valid_tpf = self.data['time_per_factor'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_tpf) > 0:
                report.append(f"  Average time per factor: {valid_tpf.mean():.4f} ms")
        
        # Memory statistics
        if 'physical_memory_mb' in self.data.columns:
            report.append("")
            report.append("MEMORY STATISTICS:")
            report.append(f"  Final memory usage: {self.data['physical_memory_mb'].iloc[-1]:.1f} MB")
            report.append(f"  Peak memory usage: {self.data['physical_memory_mb'].max():.1f} MB")
            
            if 'total_factors' in self.data.columns:
                final_memory_per_factor = self.data['physical_memory_mb'].iloc[-1] / self.data['total_factors'].iloc[-1]
                report.append(f"  Memory per factor (final): {final_memory_per_factor:.4f} MB")
        
        # Factor breakdown
        report.append("")
        report.append("FACTOR BREAKDOWN:")
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        total_factors_final = self.data['total_factors'].iloc[-1] if 'total_factors' in self.data.columns else 0
        
        for col in factor_cols:
            if col in self.data.columns:
                final_count = self.data[col].iloc[-1]
                percentage = (final_count / total_factors_final) * 100 if total_factors_final > 0 else 0
                clean_name = col.replace('total_', '').replace('_', ' ').title()
                report.append(f"  {clean_name}: {final_count} ({percentage:.1f}%)")
        
        # Save report
        report_text = '\n'.join(report)
        report_path = os.path.join(self.output_dir, 'factor_analysis_report.txt')
        
        with open(report_path, 'w') as f:
            f.write(report_text)
        
        print("\n" + report_text)
        print(f"\nDetailed report saved: {report_path}")
        
    def run_complete_analysis(self, csv_file=None):
        """Run complete factor analysis with all plots and reports"""
        print("Starting complete GNSS-IMU fusion factor analysis...")
        
        # Load data
        self.load_data(csv_file)
        
        # Generate all plots
        print("\nGenerating factor evolution plots...")
        self.plot_factor_evolution()
        
        print("\nGenerating optimization performance plots...")
        self.plot_optimization_performance()
        
        print("\nGenerating memory analysis plots...")
        self.plot_memory_analysis()
        
        # Generate summary report
        print("\nGenerating summary report...")
        self.generate_summary_report()
        
        print(f"\nAnalysis complete! All outputs saved to: {self.output_dir}")


def main():
    """Main function with command line argument support"""
    parser = argparse.ArgumentParser(description='Analyze GNSS-IMU fusion factor graph data')
    parser.add_argument('--csv', '-c', type=str, help='Path to CSV state file')
    parser.add_argument('--output', '-o', type=str, 
                       default=os.getcwd(),  # 修改默认输出目录为当前工作目录
                       help='Output directory for plots and reports (default: current directory)')
    parser.add_argument('--auto', '-a', action='store_true',
                       help='Automatically find the latest CSV file')
    
    args = parser.parse_args()
    
    # 打印当前工作目录信息
    print(f"Current working directory: {os.getcwd()}")
    print(f"Output directory: {args.output}")
    
    # Create analyzer
    analyzer = FactorAnalyzer(csv_file_path=args.csv, output_dir=args.output)
    
    # Run analysis
    try:
        if args.auto or not args.csv:
            print("Searching for latest CSV file...")
            analyzer.run_complete_analysis()
        else:
            analyzer.run_complete_analysis(args.csv)
            
    except Exception as e:
        print(f"Error during analysis: {e}")
        return 1
        
    return 0


if __name__ == "__main__":
    exit(main())
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GNSS-IMU Fusion Factor Analysis Tool

This script analyzes the factor graph growth and optimization performance
based on CSV output from the GNSS-IMU fusion system.

Author: Generated for batch_board system
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
import glob
import os
from datetime import datetime

class FactorAnalyzer:
    def __init__(self, csv_file_path=None, output_dir=None):
        """
        Initialize the Factor Analyzer
        
        Args:
            csv_file_path: Path to the state CSV file
            output_dir: Directory to save analysis plots
        """
        self.csv_file = csv_file_path
        # 修改默认输出目录为当前工作目录
        self.output_dir = output_dir or os.getcwd()
        self.data = None
        self.setup_plotting()
        
    def setup_plotting(self):
        """Setup matplotlib and seaborn plotting parameters"""
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.titlesize'] = 14
        plt.rcParams['axes.labelsize'] = 12
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        
        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
    def find_latest_csv(self, csv_dir=None):
        """Find the latest state CSV file in the given directory"""
        # 如果没有指定目录，优先在当前目录查找，然后在默认输出目录查找
        if csv_dir is None:
            search_dirs = [
                os.getcwd(),  # 当前工作目录
                "/home/jay/batch_board/output"  # 默认输出目录作为备选
            ]
        else:
            search_dirs = [csv_dir]
        
        csv_files = []
        for search_dir in search_dirs:
            pattern = os.path.join(search_dir, "gnss_imu_state_*.csv")
            found_files = glob.glob(pattern)
            csv_files.extend(found_files)
            if found_files:
                print(f"Found CSV files in directory: {search_dir}")
                break
        
        if not csv_files:
            searched_dirs = ', '.join(search_dirs)
            raise FileNotFoundError(f"No state CSV files found in directories: {searched_dirs}")
            
        # Sort by modification time, get the latest
        latest_file = max(csv_files, key=os.path.getmtime)
        print(f"Found latest CSV file: {latest_file}")
        return latest_file
        
    def load_data(self, csv_file=None):
        """Load and preprocess CSV data"""
        if csv_file:
            self.csv_file = csv_file
        elif not self.csv_file:
            self.csv_file = self.find_latest_csv()
            
        print(f"Loading data from: {self.csv_file}")
        
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.data)} records")
            
            # Convert timestamp to datetime for better plotting
            self.data['datetime'] = pd.to_datetime(self.data['timestamp'], unit='s')
            
            # Calculate relative time from start (in minutes)
            start_time = self.data['timestamp'].min()
            self.data['relative_time_min'] = (self.data['timestamp'] - start_time) / 60.0
            
            # Calculate factor growth rates
            factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                          'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
            
            for col in factor_cols:
                if col in self.data.columns:
                    self.data[f'{col}_rate'] = self.data[col].diff() / self.data['relative_time_min'].diff()
                    
            # Calculate optimization efficiency metrics
            if 'optimization_time_ms' in self.data.columns and 'total_factors' in self.data.columns:
                self.data['time_per_factor'] = self.data['optimization_time_ms'] / self.data['total_factors']
                
            print("Data preprocessing completed")
            self.print_data_summary()
            
        except Exception as e:
            raise RuntimeError(f"Error loading CSV file: {e}")
            
    def print_data_summary(self):
        """Print summary statistics of the loaded data"""
        if self.data is None:
            print("No data loaded")
            return
            
        print("\n" + "="*50)
        print("DATA SUMMARY")
        print("="*50)
        print(f"Total records: {len(self.data)}")
        print(f"Time span: {self.data['relative_time_min'].max():.1f} minutes")
        print(f"Final keyframe count: {self.data['keyframe_count'].max()}")
        
        if 'total_factors' in self.data.columns:
            print(f"Final total factors: {self.data['total_factors'].max()}")
            
        # Factor breakdown
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        print("\nFactor breakdown (final counts):")
        for col in factor_cols:
            if col in self.data.columns:
                final_count = self.data[col].max()
                percentage = (final_count / self.data['total_factors'].max()) * 100 if self.data['total_factors'].max() > 0 else 0
                print(f"  {col.replace('total_', '').replace('_', ' ').title()}: {final_count} ({percentage:.1f}%)")
                
        print("="*50)
        
    def plot_factor_evolution(self):
        """Plot the evolution of different factor types over time"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Factor Graph Evolution Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Total factors over time
        ax1 = axes[0, 0]
        if 'total_factors' in self.data.columns:
            ax1.plot(self.data['relative_time_min'], self.data['total_factors'], 
                    'b-', linewidth=2, label='Total Factors')
            ax1.set_title('Total Factors Over Time')
            ax1.set_xlabel('Time (minutes)')
            ax1.set_ylabel('Number of Factors')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            
        # Plot 2: Factor type breakdown (individual lines with colors)
        ax2 = axes[0, 1]
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        # 定义颜色和标签的映射
        factor_colors = {
            'total_gnss_pos_factors': '#1f77b4',  # 蓝色
            'total_gnss_vel_factors': '#ff7f0e',  # 橙色  
            'total_imu_factors': '#2ca02c',       # 绿色
            'total_bias_factors': '#d62728',      # 红色
            'total_prior_factors': '#9467bd'      # 紫色
        }
        
        factor_labels = {
            'total_gnss_pos_factors': 'GNSS Position',
            'total_gnss_vel_factors': 'GNSS Velocity',
            'total_imu_factors': 'IMU Preintegration',
            'total_bias_factors': 'Bias Between',
            'total_prior_factors': 'Prior Factors'
        }
        
        valid_cols = [col for col in factor_cols if col in self.data.columns]
        if valid_cols:
            for col in valid_cols:
                ax2.plot(self.data['relative_time_min'], self.data[col], 
                        color=factor_colors.get(col, 'gray'),
                        linewidth=2.5, 
                        label=factor_labels.get(col, col.replace('total_', '').title()),
                        marker='o', 
                        markersize=3,
                        alpha=0.8)
            
            ax2.set_title('Individual Factor Types Over Time')
            ax2.set_xlabel('Time (minutes)')
            ax2.set_ylabel('Number of Factors')
            ax2.legend(loc='upper left', framealpha=0.9)
            ax2.grid(True, alpha=0.3)
            
            # 添加最终数量的文本标注
            for col in valid_cols:
                final_count = self.data[col].iloc[-1]
                final_time = self.data['relative_time_min'].iloc[-1]
                ax2.annotate(f'{int(final_count)}', 
                           xy=(final_time, final_count),
                           xytext=(5, 5), 
                           textcoords='offset points',
                           fontsize=9,
                           alpha=0.8,
                           color=factor_colors.get(col, 'gray'))
            
        # Plot 3: Keyframes vs Factors
        ax3 = axes[1, 0]
        if 'keyframe_count' in self.data.columns and 'total_factors' in self.data.columns:
            scatter = ax3.scatter(self.data['keyframe_count'], self.data['total_factors'], 
                                c=self.data['relative_time_min'], cmap='viridis', alpha=0.6, s=30)
            ax3.set_title('Factors vs Keyframes')
            ax3.set_xlabel('Keyframe Count')
            ax3.set_ylabel('Total Factors')
            ax3.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax3)
            cbar.set_label('Time (minutes)')
            
            # 添加趋势线
            if len(self.data) > 2:
                z = np.polyfit(self.data['keyframe_count'], self.data['total_factors'], 1)
                p = np.poly1d(z)
                ax3.plot(self.data['keyframe_count'], p(self.data['keyframe_count']), 
                        "r--", alpha=0.8, linewidth=2,
                        label=f'Trend: {z[0]:.1f} factors/keyframe')
                ax3.legend()
            
        # Plot 4: Factor增长速度 (每轮优化新增的因子数量)
        ax4 = axes[1, 1]
        
        # 计算每轮优化新增的因子数量
        factor_cols_for_growth = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                                 'total_imu_factors', 'total_bias_factors']
        
        valid_growth_cols = [col for col in factor_cols_for_growth if col in self.data.columns]
        
        if valid_growth_cols:
            # 创建柱状图显示每轮新增的因子
            x_positions = range(len(self.data))
            bar_width = 0.8
            bottom = np.zeros(len(self.data))
            
            growth_colors = {
                'total_gnss_pos_factors': '#1f77b4',
                'total_gnss_vel_factors': '#ff7f0e', 
                'total_imu_factors': '#2ca02c',
                'total_bias_factors': '#d62728'
            }
            
            growth_labels = {
                'total_gnss_pos_factors': 'GNSS Pos',
                'total_gnss_vel_factors': 'GNSS Vel',
                'total_imu_factors': 'IMU',
                'total_bias_factors': 'Bias'
            }
            
            for col in valid_growth_cols:
                # 计算每轮新增数量 (当前值 - 前一个值)
                factor_diff = self.data[col].diff().fillna(self.data[col].iloc[0])
                factor_diff = factor_diff.clip(lower=0)  # 确保非负值
                
                if factor_diff.sum() > 0:  # 只有当有增长时才绘制
                    ax4.bar(x_positions, factor_diff, 
                           bottom=bottom,
                           width=bar_width,
                           label=growth_labels.get(col, col.replace('total_', '').title()),
                           color=growth_colors.get(col, 'gray'),
                           alpha=0.8)
                    bottom += factor_diff
            
            ax4.set_title('Factors Added Per Optimization Round')
            ax4.set_xlabel('Optimization Round')
            ax4.set_ylabel('New Factors Added')
            ax4.legend(loc='upper right', framealpha=0.9)
            ax4.grid(True, alpha=0.3, axis='y')
            
            # 设置x轴标签，每隔几个显示一次避免拥挤
            if len(self.data) > 20:
                step = len(self.data) // 10
                ax4.set_xticks(range(0, len(self.data), step))
                ax4.set_xticklabels(range(0, len(self.data), step))
            
            # 添加总数统计
            total_new = bottom.sum()
            ax4.text(0.02, 0.98, f'Total new factors: {int(total_new)}', 
                    transform=ax4.transAxes, 
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'factor_evolution.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Factor evolution plot saved: {output_path}")
        plt.show()
        
    def plot_optimization_performance(self):
        """Plot optimization performance metrics"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Optimization Performance Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Optimization time over iterations
        ax1 = axes[0, 0]
        if 'optimization_time_ms' in self.data.columns:
            ax1.plot(self.data['seq'], self.data['optimization_time_ms'], 
                    'r-', linewidth=2, alpha=0.7)
            ax1.set_title('Optimization Time per Iteration')
            ax1.set_xlabel('Iteration')
            ax1.set_ylabel('Time (ms)')
            ax1.grid(True, alpha=0.3)
            
            # Add trend line
            if len(self.data) > 1:
                z = np.polyfit(self.data['seq'], self.data['optimization_time_ms'], 1)
                p = np.poly1d(z)
                ax1.plot(self.data['seq'], p(self.data['seq']), "--", alpha=0.8, color='darkred', 
                        label=f'Trend (slope: {z[0]:.3f} ms/iter)')
                ax1.legend()
        
        # Plot 2: Time per factor
        ax2 = axes[0, 1]
        if 'time_per_factor' in self.data.columns:
            valid_data = self.data['time_per_factor'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_data) > 0:
                ax2.plot(self.data.loc[valid_data.index, 'relative_time_min'], valid_data, 
                        'g-', linewidth=2, alpha=0.7)
                ax2.set_title('Optimization Efficiency (Time per Factor)')
                ax2.set_xlabel('Time (minutes)')
                ax2.set_ylabel('ms per Factor')
                ax2.grid(True, alpha=0.3)
        
        # Plot 3: Graph size vs optimization time
        ax3 = axes[1, 0]
        if 'graph_size' in self.data.columns and 'optimization_time_ms' in self.data.columns:
            scatter = ax3.scatter(self.data['graph_size'], self.data['optimization_time_ms'], 
                                c=self.data['relative_time_min'], cmap='plasma', alpha=0.6)
            ax3.set_title('Graph Size vs Optimization Time')
            ax3.set_xlabel('Graph Size (factors per iteration)')
            ax3.set_ylabel('Optimization Time (ms)')
            ax3.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax3)
            cbar.set_label('Time (minutes)')
        
        # Plot 4: High-frequency pose output rate
        ax4 = axes[1, 1]
        if 'high_freq_pose_count' in self.data.columns:
            # Calculate pose rate
            time_diff = self.data['relative_time_min'].diff()
            pose_rate = self.data['high_freq_pose_count'] / (time_diff * 60)  # poses per second
            valid_rates = pose_rate.replace([np.inf, -np.inf], np.nan).dropna()
            
            if len(valid_rates) > 1:
                ax4.plot(self.data.loc[valid_rates.index, 'relative_time_min'], valid_rates, 
                        'purple', linewidth=2, alpha=0.7)
                ax4.set_title('High-Frequency Pose Output Rate')
                ax4.set_xlabel('Time (minutes)')
                ax4.set_ylabel('Poses per Second')
                ax4.grid(True, alpha=0.3)
                
                # Add average line
                avg_rate = valid_rates.mean()
                ax4.axhline(y=avg_rate, color='red', linestyle='--', alpha=0.8, 
                           label=f'Average: {avg_rate:.1f} Hz')
                ax4.legend()
        
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'optimization_performance.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Optimization performance plot saved: {output_path}")
        plt.show()
        
    def plot_memory_analysis(self):
        """Plot memory usage analysis"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        memory_cols = ['virtual_memory_mb', 'physical_memory_mb', 'peak_memory_mb']
        available_memory_cols = [col for col in memory_cols if col in self.data.columns]
        
        if not available_memory_cols:
            print("No memory data available for analysis")
            return
            
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Memory Usage Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Memory usage over time
        ax1 = axes[0, 0]
        for col in available_memory_cols:
            ax1.plot(self.data['relative_time_min'], self.data[col], 
                    linewidth=2, label=col.replace('_', ' ').title(), alpha=0.8)
        ax1.set_title('Memory Usage Over Time')
        ax1.set_xlabel('Time (minutes)')
        ax1.set_ylabel('Memory (MB)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Memory vs factors
        ax2 = axes[0, 1]
        if 'physical_memory_mb' in self.data.columns and 'total_factors' in self.data.columns:
            scatter = ax2.scatter(self.data['total_factors'], self.data['physical_memory_mb'], 
                                c=self.data['relative_time_min'], cmap='coolwarm', alpha=0.6)
            ax2.set_title('Memory vs Total Factors')
            ax2.set_xlabel('Total Factors')
            ax2.set_ylabel('Physical Memory (MB)')
            ax2.grid(True, alpha=0.3)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax2)
            cbar.set_label('Time (minutes)')
        
        # Plot 3: Memory growth rate
        ax3 = axes[1, 0]
        if 'memory_growth_mb' in self.data.columns:
            valid_growth = self.data['memory_growth_mb'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_growth) > 1:
                ax3.plot(self.data.loc[valid_growth.index, 'relative_time_min'], valid_growth, 
                        'orange', linewidth=2, alpha=0.7)
                ax3.set_title('Memory Growth Rate')
                ax3.set_xlabel('Time (minutes)')
                ax3.set_ylabel('Memory Growth (MB)')
                ax3.grid(True, alpha=0.3)
                ax3.axhline(y=0, color='red', linestyle='--', alpha=0.5)
        
        # Plot 4: Memory efficiency
        ax4 = axes[1, 1]
        if 'physical_memory_mb' in self.data.columns and 'total_factors' in self.data.columns:
            memory_per_factor = self.data['physical_memory_mb'] / self.data['total_factors']
            valid_efficiency = memory_per_factor.replace([np.inf, -np.inf], np.nan).dropna()
            
            if len(valid_efficiency) > 1:
                ax4.plot(self.data.loc[valid_efficiency.index, 'relative_time_min'], 
                        valid_efficiency, 'brown', linewidth=2, alpha=0.7)
                ax4.set_title('Memory Efficiency (MB per Factor)')
                ax4.set_xlabel('Time (minutes)')
                ax4.set_ylabel('MB per Factor')
                ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        output_path = os.path.join(self.output_dir, 'memory_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Memory analysis plot saved: {output_path}")
        plt.show()
        
    def generate_summary_report(self):
        """Generate a comprehensive summary report"""
        if self.data is None:
            raise RuntimeError("No data loaded. Call load_data() first.")
            
        report = []
        report.append("GNSS-IMU FUSION FACTOR ANALYSIS REPORT")
        report.append("=" * 50)
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Data file: {os.path.basename(self.csv_file)}")
        report.append("")
        
        # Basic statistics
        report.append("BASIC STATISTICS:")
        report.append(f"  Total optimization iterations: {len(self.data)}")
        report.append(f"  Total runtime: {self.data['relative_time_min'].max():.1f} minutes")
        report.append(f"  Final keyframe count: {self.data['keyframe_count'].max()}")
        
        if 'total_factors' in self.data.columns:
            report.append(f"  Final total factors: {self.data['total_factors'].max()}")
            avg_factors_per_keyframe = self.data['total_factors'].max() / self.data['keyframe_count'].max()
            report.append(f"  Average factors per keyframe: {avg_factors_per_keyframe:.1f}")
        
        # Performance statistics
        report.append("")
        report.append("PERFORMANCE STATISTICS:")
        if 'optimization_time_ms' in self.data.columns:
            report.append(f"  Average optimization time: {self.data['optimization_time_ms'].mean():.2f} ms")
            report.append(f"  Max optimization time: {self.data['optimization_time_ms'].max():.2f} ms")
            report.append(f"  Min optimization time: {self.data['optimization_time_ms'].min():.2f} ms")
        
        if 'time_per_factor' in self.data.columns:
            valid_tpf = self.data['time_per_factor'].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid_tpf) > 0:
                report.append(f"  Average time per factor: {valid_tpf.mean():.4f} ms")
        
        # Memory statistics
        if 'physical_memory_mb' in self.data.columns:
            report.append("")
            report.append("MEMORY STATISTICS:")
            report.append(f"  Final memory usage: {self.data['physical_memory_mb'].iloc[-1]:.1f} MB")
            report.append(f"  Peak memory usage: {self.data['physical_memory_mb'].max():.1f} MB")
            
            if 'total_factors' in self.data.columns:
                final_memory_per_factor = self.data['physical_memory_mb'].iloc[-1] / self.data['total_factors'].iloc[-1]
                report.append(f"  Memory per factor (final): {final_memory_per_factor:.4f} MB")
        
        # Factor breakdown
        report.append("")
        report.append("FACTOR BREAKDOWN:")
        factor_cols = ['total_gnss_pos_factors', 'total_gnss_vel_factors', 
                      'total_imu_factors', 'total_bias_factors', 'total_prior_factors']
        
        total_factors_final = self.data['total_factors'].iloc[-1] if 'total_factors' in self.data.columns else 0
        
        for col in factor_cols:
            if col in self.data.columns:
                final_count = self.data[col].iloc[-1]
                percentage = (final_count / total_factors_final) * 100 if total_factors_final > 0 else 0
                clean_name = col.replace('total_', '').replace('_', ' ').title()
                report.append(f"  {clean_name}: {final_count} ({percentage:.1f}%)")
        
        # Save report
        report_text = '\n'.join(report)
        report_path = os.path.join(self.output_dir, 'factor_analysis_report.txt')
        
        with open(report_path, 'w') as f:
            f.write(report_text)
        
        print("\n" + report_text)
        print(f"\nDetailed report saved: {report_path}")
        
    def run_complete_analysis(self, csv_file=None):
        """Run complete factor analysis with all plots and reports"""
        print("Starting complete GNSS-IMU fusion factor analysis...")
        
        # Load data
        self.load_data(csv_file)
        
        # Generate all plots
        print("\nGenerating factor evolution plots...")
        self.plot_factor_evolution()
        
        print("\nGenerating optimization performance plots...")
        self.plot_optimization_performance()
        
        print("\nGenerating memory analysis plots...")
        self.plot_memory_analysis()
        
        # Generate summary report
        print("\nGenerating summary report...")
        self.generate_summary_report()
        
        print(f"\nAnalysis complete! All outputs saved to: {self.output_dir}")


def main():
    """Main function with command line argument support"""
    parser = argparse.ArgumentParser(description='Analyze GNSS-IMU fusion factor graph data')
    parser.add_argument('--csv', '-c', type=str, help='Path to CSV state file')
    parser.add_argument('--output', '-o', type=str, 
                       default=os.getcwd(),  # 修改默认输出目录为当前工作目录
                       help='Output directory for plots and reports (default: current directory)')
    parser.add_argument('--auto', '-a', action='store_true',
                       help='Automatically find the latest CSV file')
    
    args = parser.parse_args()
    
    # 打印当前工作目录信息
    print(f"Current working directory: {os.getcwd()}")
    print(f"Output directory: {args.output}")
    
    # Create analyzer
    analyzer = FactorAnalyzer(csv_file_path=args.csv, output_dir=args.output)
    
    # Run analysis
    try:
        if args.auto or not args.csv:
            print("Searching for latest CSV file...")
            analyzer.run_complete_analysis()
        else:
            analyzer.run_complete_analysis(args.csv)
            
    except Exception as e:
        print(f"Error during analysis: {e}")
        return 1
        
    return 0


if __name__ == "__main__":
    exit(main())
