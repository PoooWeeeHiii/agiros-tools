import collections
import heapq
import os
import re
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Pattern, Sequence, Set, Tuple


class PackageDepGraph:
    """Represents dependencies between Debian packages discovered from control files."""

    def __init__(self, package_dirs: Mapping[str, Path], order_hint: Optional[Mapping[str, int]] = None):
        self.package_dirs = {name: path for name, path in package_dirs.items()}
        self.order_hint = dict(order_hint or {})
        self.nodes: Set[str] = set(self.package_dirs.keys())
        self.adj: Dict[str, Set[str]] = collections.defaultdict(set)
        self.rev: Dict[str, Set[str]] = collections.defaultdict(set)
        self.unresolved: Dict[str, Set[str]] = collections.defaultdict(set)
        for pkg in self.nodes:
            self.adj.setdefault(pkg, set())
            self.rev.setdefault(pkg, set())

    def add_edge(self, prereq: str, dependent: str) -> None:
        if prereq == dependent:
            return
        self.adj[prereq].add(dependent)
        self.rev[dependent].add(prereq)

    def build_from_control_dirs(self, depends_field_names: Optional[Iterable[str]] = None) -> None:
        fields = list(depends_field_names) if depends_field_names else [
            "Depends",
            "Build-Depends",
            "Build-Depends-Indep",
            "Build-Depends-Arch",
        ]
        pkgname_re = re.compile(r"([A-Za-z0-9+_.:-]+)")
        for pkg, base_path in self.package_dirs.items():
            ctrl_path = base_path / "debian" / "control"
            if not ctrl_path.is_file():
                continue
            try:
                content = ctrl_path.read_text(encoding="utf-8")
            except Exception:
                continue
            for stanza in _split_paragraphs(content):
                for field in fields:
                    raw_value = stanza.get(field)
                    if not raw_value:
                        continue
                    for dep_pkg in _parse_depends(raw_value, pkgname_re):
                        if dep_pkg in self.package_dirs and dep_pkg != pkg:
                            self.add_edge(dep_pkg, pkg)
                        elif dep_pkg not in self.package_dirs:
                            self.unresolved[pkg].add(dep_pkg)

    def topo_sort(self, subset: Optional[Sequence[str]] = None, include_dependencies: bool = False) -> List[str]:
        if subset is None:
            working_nodes = set(self.nodes)
        else:
            working_nodes = set()
            stack = list(subset)
            while stack:
                node = stack.pop()
                if node in working_nodes:
                    continue
                if node not in self.nodes:
                    raise KeyError(f"Package {node} not known in dependency graph")
                working_nodes.add(node)
                for predecessor in self.rev.get(node, ()):
                    if predecessor not in working_nodes:
                        stack.append(predecessor)
        in_degree = {node: 0 for node in working_nodes}
        for node in working_nodes:
            for follower in self.adj.get(node, ()):
                if follower in working_nodes:
                    in_degree[follower] += 1
        default_priority = len(self.order_hint) + len(working_nodes) + 5
        queue: List[Tuple[int, str]] = []
        for node, degree in in_degree.items():
            if degree == 0:
                priority = self.order_hint.get(node, default_priority)
                heapq.heappush(queue, (priority, node))
        sorted_nodes: List[str] = []
        while queue:
            _, node = heapq.heappop(queue)
            sorted_nodes.append(node)
            for follower in self.adj.get(node, ()):
                if follower not in working_nodes:
                    continue
                in_degree[follower] -= 1
                if in_degree[follower] == 0:
                    priority = self.order_hint.get(follower, default_priority)
                    heapq.heappush(queue, (priority, follower))
        if len(sorted_nodes) != len(working_nodes):
            remaining = working_nodes - set(sorted_nodes)
            raise ValueError(f"Cycle detected among: {', '.join(sorted(remaining))}")
        if subset is None or include_dependencies:
            return sorted_nodes
        subset_set = set(subset)
        return [node for node in sorted_nodes if node in subset_set]


def discover_debian_package_dirs(code_dir: Path, existing: Sequence[Tuple[str, Path]]) -> Dict[str, Path]:
    """Return package directories that contain debian/control, keyed by package name."""
    packages: Dict[str, Path] = {}
    for name, path in existing:
        try:
            packages[name] = path.expanduser().resolve()
        except Exception:
            packages[name] = path
    for pkg_dir in _scan_package_dirs(code_dir):
        packages.setdefault(pkg_dir.name, pkg_dir)
    return packages


def sort_packages_with_dependencies(
    package_dirs: Mapping[str, Path],
    target_packages: Sequence[str],
    order_hint: Optional[Mapping[str, int]] = None,
) -> Tuple[List[str], Set[str]]:
    """Return dependency-aware ordering plus unresolved dependency names."""
    graph = PackageDepGraph(package_dirs, order_hint=order_hint)
    graph.build_from_control_dirs()
    unresolved: Set[str] = set()
    for deps in graph.unresolved.values():
        unresolved.update(deps)
    return graph.topo_sort(target_packages, include_dependencies=True), unresolved


def compute_series_toposort(
    package_dirs: Mapping[str, Path],
    target_packages: Sequence[str],
    order_hint: Optional[Mapping[str, int]] = None,
) -> Tuple[List[List[str]], Set[str]]:
    """
    将待构建包按弱连通分量拆分为若干“系列”，系列内部保持拓扑顺序。

    返回值:
      - series: List[List[str]]，每个子列表是一个弱连通分量内的拓扑序；
                系列按规模降序排列，便于并行调度时优先处理大块任务。
      - unresolved: Set[str]，未在本地源码中找到的依赖名。
    """
    graph = PackageDepGraph(package_dirs, order_hint=order_hint)
    graph.build_from_control_dirs()

    unresolved: Set[str] = set()
    for deps in graph.unresolved.values():
        unresolved.update(deps)

    topo_all = graph.topo_sort(target_packages, include_dependencies=True)
    if not topo_all:
        return [], unresolved
    topo_index = {name: idx for idx, name in enumerate(topo_all)}

    # 构建无向邻接表用于弱连通分量拆分
    relevant = set(topo_all)
    undirected: Dict[str, Set[str]] = {node: set() for node in relevant}
    for node in relevant:
        for follower in graph.adj.get(node, ()):
            if follower in relevant:
                undirected[node].add(follower)
                undirected[follower].add(node)
        for predecessor in graph.rev.get(node, ()):
            if predecessor in relevant:
                undirected[node].add(predecessor)
                undirected[predecessor].add(node)

    series: List[List[str]] = []
    visited: Set[str] = set()
    # 以 topo 序做遍历顺序，确保组件内后续排序稳定
    for node in topo_all:
        if node in visited:
            continue
        stack = [node]
        comp: List[str] = []
        while stack:
            cur = stack.pop()
            if cur in visited:
                continue
            visited.add(cur)
            comp.append(cur)
            stack.extend(undirected[cur] - visited)
        comp.sort(key=lambda n: topo_index[n])  # 组件内保持拓扑序
        series.append(comp)

    # 按组件大小降序，大小相同则以最早拓扑位置作为稳定排序
    series.sort(key=lambda comp: (-len(comp), topo_index.get(comp[0], 0)))
    return series, unresolved


def _scan_package_dirs(code_dir: Path, max_depth: int = 3) -> List[Path]:
    if not code_dir.exists():
        return []
    discovered: List[Path] = []
    code_dir = code_dir.expanduser()
    for root, dirs, files in os.walk(code_dir):
        root_path = Path(root)
        rel_parts = root_path.relative_to(code_dir).parts
        if len(rel_parts) > max_depth:
            dirs[:] = []
            continue
        ctrl = root_path / "debian" / "control"
        if ctrl.is_file():
            try:
                resolved = root_path.resolve()
            except Exception:
                resolved = root_path
            discovered.append(resolved)
            dirs[:] = []
    return discovered


def _split_paragraphs(content: str) -> List[Dict[str, str]]:
    paragraphs: List[Dict[str, str]] = []
    current: Dict[str, str] = {}
    current_key: Optional[str] = None
    for raw_line in content.splitlines():
        line = raw_line.rstrip()
        if not line.strip():
            if current:
                paragraphs.append(current)
                current = {}
                current_key = None
            continue
        if line[0].isspace():
            if current_key:
                current[current_key] += " " + line.strip()
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        current_key = key.strip()
        current[current_key] = value.strip()
    if current:
        paragraphs.append(current)
    return paragraphs


def _parse_depends(value: str, pkgname_re: Pattern[str]) -> List[str]:
    deps: List[str] = []
    for part in value.split(","):
        token = part.split("|")[0].strip()
        token = token.split("(")[0].strip()
        token = token.split("[")[0].strip()
        token = token.split("{")[0].strip()
        if ":" in token:
            token = token.split(":")[0].strip()
        match = pkgname_re.match(token)
        if not match:
            continue
        deps.append(match.group(1))
    return deps
