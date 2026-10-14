#!/usr/bin/env python3
"""
generate_ppt_table_images.py
Generates 3 PPT-ready table images (one for each speed: 3.5 m/s, 5.5 m/s, 7.5 m/s)
with pure white background, crisp black text, clean borders,
and publication-quality typography ready for PowerPoint presentations.
"""

import os
import sys
import scipy.io
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def load_simulation_results():
    mat_path = 'simulation_results/all_12_simulations_summary.mat'
    if not os.path.exists(mat_path):
        print(f"Error: {mat_path} not found.")
        sys.exit(1)
    data = scipy.io.loadmat(mat_path, simplify_cells=True)
    return data['summary_metrics']

def render_table_image(speed_val, model_data, output_paths):
    """
    Renders a clean, pure white-background, high-DPI table image optimized for PowerPoint slides.
    """
    # 12 inches x 4.5 inches with 300 DPI gives 3600 x 1350 px ultra-crisp resolution
    fig, ax = plt.subplots(figsize=(11.5, 4.3), dpi=300)
    fig.patch.set_facecolor('#FFFFFF')
    ax.set_facecolor('#FFFFFF')
    ax.axis('off')

    # Title & Subtitle in black/dark charcoal
    title_text = f"Simulation Tracking Performance Summary  (v = {speed_val:.1f} m/s)"
    subtitle_text = "Comparison of 4 Flight Control Models on 3D Curvature Trajectory (Path 2, dt = 0.01s)"
    
    ax.text(0.03, 0.93, title_text, fontsize=15, fontweight='bold', color='#000000', ha='left', va='top')
    ax.text(0.03, 0.84, subtitle_text, fontsize=10, color='#333333', ha='left', va='top')

    # Table columns
    col_labels = ["No.", "Control Model Architecture", "RMSE [m]", "MAE [m]", "MAX Error [m]", "RMSE Reduction"]
    # Column width distribution: total = 0.94
    col_widths = [0.06, 0.40, 0.12, 0.12, 0.12, 0.12]
    
    # Calculate baseline RMSE for percentage improvement
    base_rmse = model_data[0]['rmse']

    table_rows = []
    for idx, m in enumerate(model_data):
        m_num = f"#{idx+1}"
        m_name = m['name']
        rmse_str = f"{m['rmse']:.4f} m"
        mae_str = f"{m['mae']:.4f} m"
        max_str = f"{m['max']:.4f} m"
        
        if idx == 0:
            imp_str = "Baseline"
        else:
            pct = ((base_rmse - m['rmse']) / base_rmse) * 100.0
            imp_str = f"-{pct:.1f}%" if pct >= 0 else f"+{abs(pct):.1f}%"
            
        table_rows.append([m_num, m_name, rmse_str, mae_str, max_str, imp_str])

    # Table layout
    y_start = 0.72
    row_height = 0.115
    x_start = 0.03
    total_width = 0.94

    # Header Row
    header_rect = patches.FancyBboxPatch(
        (x_start, y_start - row_height), total_width, row_height,
        boxstyle="round,pad=0.003,rounding_size=0.012",
        linewidth=1.2, edgecolor='#000000', facecolor='#F3F4F6'
    )
    ax.add_patch(header_rect)

    # Header text
    curr_x = x_start
    for col_idx, (label, width) in enumerate(zip(col_labels, col_widths)):
        if col_idx == 0:
            cell_x = curr_x + width / 2.0
            ha = 'center'
        elif col_idx == 1:
            cell_x = curr_x + 0.02
            ha = 'left'
        else:
            cell_x = curr_x + width / 2.0
            ha = 'center'
            
        ax.text(cell_x, y_start - row_height/2.0, label,
                fontsize=10, fontweight='bold', color='#000000',
                ha=ha, va='center')
        curr_x += width

    # Data Rows
    curr_y = y_start - row_height
    for row_idx, row in enumerate(table_rows):
        row_y = curr_y - row_height
        is_ppo = (row_idx == 3) # Model 4 (PPO)
        
        if is_ppo:
            bg_color = '#F0FDF4' # Subtle light green tint or clean white
            border_color = '#15803D'
            text_color = '#000000'
            font_weight = 'bold'
            line_w = 1.6
        else:
            bg_color = '#FFFFFF' if row_idx % 2 == 0 else '#F9FAFB'
            border_color = '#D1D5DB'
            text_color = '#111827'
            font_weight = 'normal'
            line_w = 0.9

        # Row box
        row_rect = patches.FancyBboxPatch(
            (x_start, row_y), total_width, row_height,
            boxstyle="round,pad=0.003,rounding_size=0.008",
            linewidth=line_w,
            edgecolor=border_color, facecolor=bg_color
        )
        ax.add_patch(row_rect)

        # Draw cell contents
        curr_x = x_start
        for col_idx, (val, width) in enumerate(zip(row, col_widths)):
            if col_idx == 0:
                cell_x = curr_x + width / 2.0
                ha = 'center'
            elif col_idx == 1:
                cell_x = curr_x + 0.02
                ha = 'left'
            else:
                cell_x = curr_x + width / 2.0
                ha = 'center'
            
            c_color = text_color
            weight = font_weight
            
            if is_ppo and col_idx == 1:
                val = val + "  (Best Tracking)"
            if is_ppo and col_idx == 5:
                val = val + " ▼"
                c_color = '#15803D'
                weight = 'bold'

            ax.text(cell_x, row_y + row_height/2.0, val,
                    fontsize=9.5, fontweight=weight, color=c_color,
                    ha=ha, va='center')
            curr_x += width

        curr_y = row_y

    # Bottom footnote
    footnote = (
        f"* Simulation Specifications: Mass m = 1.25 kg, dt = 0.01 s, N_sub = 10 (RL 10 Hz action update).\n"
        f"* Metric Definition: RMSE = sqrt(mean(||e||^2)), MAE = mean(||e||), MAX = max(||e||), e(t) = pos(t) - pos_des(t)."
    )
    ax.text(0.03, 0.05, footnote, fontsize=8, color='#4B5563', ha='left', va='bottom', linespacing=1.3)

    plt.tight_layout()
    for p in output_paths:
        os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
        plt.savefig(p, dpi=300, facecolor='#FFFFFF', edgecolor='none', bbox_inches='tight')
        print(f"Saved PPT Table Image: {p}")
    plt.close()

def render_all_speeds_table(metrics, speeds):
    """
    Renders an overarching summary table with all 3 speeds side-by-side.
    """
    fig, ax = plt.subplots(figsize=(11.5, 6.4), dpi=300)
    fig.patch.set_facecolor('#FFFFFF')
    ax.set_facecolor('#FFFFFF')
    ax.axis('off')

    title_text = "Overall Flight Control Performance Comparison Across All Speeds"
    subtitle_text = "Complete 12-Run Benchmark (3 Speeds × 4 Models): RMSE, MAE, and MAX Errors"
    
    ax.text(0.03, 0.95, title_text, fontsize=15, fontweight='bold', color='#000000', ha='left', va='top')
    ax.text(0.03, 0.89, subtitle_text, fontsize=10, color='#333333', ha='left', va='top')

    col_labels = ["Velocity", "Control Model Architecture", "RMSE [m]", "MAE [m]", "MAX Error [m]", "RMSE Reduction"]
    col_widths = [0.12, 0.38, 0.11, 0.11, 0.11, 0.11]
    
    y_start = 0.82
    row_height = 0.053
    x_start = 0.03
    total_width = 0.94

    # Draw Header
    header_rect = patches.FancyBboxPatch(
        (x_start, y_start - row_height), total_width, row_height,
        boxstyle="round,pad=0.003,rounding_size=0.012",
        linewidth=1.2, edgecolor='#000000', facecolor='#F3F4F6'
    )
    ax.add_patch(header_rect)

    curr_x = x_start
    for col_idx, (label, width) in enumerate(zip(col_labels, col_widths)):
        if col_idx == 0:
            cell_x = curr_x + width / 2.0
            ha = 'center'
        elif col_idx == 1:
            cell_x = curr_x + 0.02
            ha = 'left'
        else:
            cell_x = curr_x + width / 2.0
            ha = 'center'
            
        ax.text(cell_x, y_start - row_height/2.0, label,
                fontsize=9.2, fontweight='bold', color='#000000',
                ha=ha, va='center')
        curr_x += width

    curr_y = y_start - row_height
    model_names = [
        "1. Hovering Fixed PID (Baseline)",
        "2. Feedforward Only",
        "3. Applied Gain Scheduling",
        "4. Proposed: Drone_LPV + Residual PPO"
    ]

    for v in speeds:
        v_key = f"v_{str(v).replace('.', '_')}"
        v_data = metrics[v_key]
        m_keys = ['Model1', 'Model2', 'Model3', 'Model4']
        base_rmse = float(v_data['Model1']['RMSE'])
        
        for m_idx, (m_k, m_name) in enumerate(zip(m_keys, model_names)):
            row_y = curr_y - row_height
            is_ppo = (m_idx == 3)
            
            rmse = float(v_data[m_k]['RMSE'])
            mae = float(v_data[m_k]['MAE'])
            max_err = float(v_data[m_k]['MAX'])
            
            if m_idx == 0:
                imp_str = "Baseline"
            else:
                pct = ((base_rmse - rmse) / base_rmse) * 100.0
                imp_str = f"-{pct:.1f}%" if pct >= 0 else f"+{abs(pct):.1f}%"
            
            bg_color = '#F0FDF4' if is_ppo else ('#FFFFFF' if (m_idx % 2 == 0) else '#F9FAFB')
            border_color = '#15803D' if is_ppo else '#D1D5DB'
            text_color = '#000000' if is_ppo else '#1F2937'
            font_weight = 'bold' if is_ppo else 'normal'

            row_rect = patches.FancyBboxPatch(
                (x_start, row_y), total_width, row_height,
                boxstyle="round,pad=0.003,rounding_size=0.006",
                linewidth=1.3 if is_ppo else 0.8,
                edgecolor=border_color, facecolor=bg_color
            )
            ax.add_patch(row_rect)

            v_label = f"v = {v:.1f} m/s" if m_idx == 0 else ""
            if is_ppo:
                m_name = m_name + " ★"
                
            row_vals = [
                v_label,
                m_name,
                f"{rmse:.4f} m",
                f"{mae:.4f} m",
                f"{max_err:.4f} m",
                imp_str
            ]

            curr_x = x_start
            for col_idx, (val, width) in enumerate(zip(row_vals, col_widths)):
                if col_idx == 0:
                    cell_x = curr_x + width / 2.0
                    ha = 'center'
                elif col_idx == 1:
                    cell_x = curr_x + 0.02
                    ha = 'left'
                else:
                    cell_x = curr_x + width / 2.0
                    ha = 'center'
                
                c_color = text_color
                weight = font_weight
                if col_idx == 0:
                    weight = 'bold'
                    c_color = '#000000'
                if col_idx == 5 and is_ppo:
                    c_color = '#15803D'
                    weight = 'bold'

                ax.text(cell_x, row_y + row_height/2.0, val,
                        fontsize=8.8, fontweight=weight, color=c_color,
                        ha=ha, va='center')
                curr_x += width

            curr_y = row_y

    footnote = "* All evaluations tested on Path 2 trajectory with attitude command shaping low-pass filter (alpha = 0.25)."
    ax.text(0.03, 0.03, footnote, fontsize=8, color='#4B5563', ha='left', va='bottom')

    plt.tight_layout()
    plt.savefig("table_summary_all_speeds.png", dpi=300, facecolor='#FFFFFF', bbox_inches='tight')
    plt.savefig("simulation_results/table_summary_all_speeds.png", dpi=300, facecolor='#FFFFFF', bbox_inches='tight')
    print("Saved Overall Table Image: table_summary_all_speeds.png")
    plt.close()

def main():
    metrics = load_simulation_results()
    speeds = [3.5, 5.5, 7.5]
    
    for v in speeds:
        v_key = f"v_{str(v).replace('.', '_')}"
        if v_key not in metrics:
            print(f"Warning: {v_key} not found in summary metrics.")
            continue
            
        v_data = metrics[v_key]
        model_list = [
            {
                'name': 'Hovering Fixed PID (Baseline)',
                'rmse': float(v_data['Model1']['RMSE']),
                'mae': float(v_data['Model1']['MAE']),
                'max': float(v_data['Model1']['MAX'])
            },
            {
                'name': 'Feedforward Only',
                'rmse': float(v_data['Model2']['RMSE']),
                'mae': float(v_data['Model2']['MAE']),
                'max': float(v_data['Model2']['MAX'])
            },
            {
                'name': 'Applied Gain Scheduling',
                'rmse': float(v_data['Model3']['RMSE']),
                'mae': float(v_data['Model3']['MAE']),
                'max': float(v_data['Model3']['MAX'])
            },
            {
                'name': 'Proposed: Drone_LPV + Residual PPO',
                'rmse': float(v_data['Model4']['RMSE']),
                'mae': float(v_data['Model4']['MAE']),
                'max': float(v_data['Model4']['MAX'])
            }
        ]
        
        out_paths = [
            f"table_summary_v{str(v).replace('.', '_')}.png",
            f"simulation_results/v={v:.1f}_ms/table_summary_v{str(v).replace('.', '_')}.png"
        ]
        render_table_image(v, model_list, out_paths)

    render_all_speeds_table(metrics, speeds)

if __name__ == '__main__':
    main()
