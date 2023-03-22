# vscode配置MATLAB

> zhangqq  
> Mar-21, 2023  
> Chongqing

---

## 安装vscode扩展

| Matlab                      | 代码高亮、代码补全（此功能效果不佳，部分代码无法自动补全）、实时语法检查 |
| --------------------------- | ------------------------------------------------------------ |
| Matlab Interactive Terminal | 在Vscode的终端中运行m文件与Matlab命令行                      |
| Matlab Snippets             | 代码补全（对插件Matlab代码补全功能的补充）                   |
| matlab-formatter            | 代码格式化                                                   |

![image-20230321150110982](F:\carla\New folder (2)\img\vscode-matlab)

## setting.json

```
    //* matlab相关插件设置  begin +++++++++++++++++++++++++++++++++++++
    "editor.snippetSuggestions": "top", // 在其它建议上方显示代码片段建议
    "matlab.matlabpath": "F:/zqq/R2021a/bin/matlab.exe",
    "matlab.mlintpath": "F:/zqq/R2021a/bin/win64/mlint.exe",
  	//* matlab插件相关 设置  end   -------------------------------------
```

code runner

```
"code-runner.executorMap": {
      "matlab": "cd $dir && matlab -nosplash -nodesktop -r $fileNameWithoutExt"
    },
```

