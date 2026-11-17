# 可直接导入的工业工位

这些 JSON 使用 RoboStudio 自身的场景格式。在 App 顶部打开“场景库”，选择文件，
查看验证预览后载入；也可在工业工位卡片中直接载入并运行同一配置。

- part_loading：圆柱零件上下料。
- incoming_sorting：三色物料分拣，静态来料区。
- case_stacking：三层成品码垛。
- partition_transfer：隔板避障转运。

单位为 m、rad、kg，size 使用半尺寸。场景描述初始布局，不包含当前速度或任务进度。
通过界面编辑、导出即可创建自己的文件；运行记录 JSON 不能当作场景导入。
