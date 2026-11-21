#!/bin/bash
# build_report.sh -- 把生成好的表格/结果拼进 report.md
cd /home/jacky/sim/val7
cp report_source.md report_backup.md
head -6 report_backup.md > report_new.md
echo "" >> report_new.md
cat total_table.md >> report_new.md
echo "" >> report_new.md
sed -n '8,271p' report_backup.md >> report_new.md
sed -n '272,288p' report_backup.md >> report_new.md
cat s4_results.md >> report_new.md
sed -n '289,298p' report_backup.md >> report_new.md
cat s5_results.md >> report_new.md
sed -n '299,$p' report_backup.md >> report_new.md
cat sugg_section.md >> report_new.md
cat fail_list.md >> report_new.md
cat notdone.md >> report_new.md
mv report_new.md report.md
wc -l report.md
